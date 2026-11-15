// Multi-target, multi-sensor ground-truth regression: simulator output ->
// C++ decoder -> Engine (scan-grouped, as the ingest path delivers it),
// scored against the simulator's truth.
//
// Fixtures: tests/fixtures/scenarios/{crossing_pair,close_pass,enter_leave}/,
// regenerated with
//   cd sim && python3 -m sifted_sim.generate <name> ../tests/fixtures/scenarios/<name>
// The engine sees only the sensor table and the frames. Truth lists, once per
// second, every target that exists, flagged when it is inside radar coverage
// (truth.csv layout 2, with a
// target_id and a covered flag; see tests/fixtures/README.md).
//
// Scoring (CLEAR-MOT style; Bernardin and Stiefelhagen, 2008). At each truth
// time T, after feeding every frame with time <= T:
//   * reported tracks are the confirmed and coasting tracks, extrapolated to T;
//   * they are matched one-to-one to the truth targets present at T. As in
//     CLEAR-MOT, a target's previous match is kept while that track is still
//     reported and within kMatchDistanceM (so two targets passing through the
//     same point do not flip the score); the rest are matched by a
//     minimum-total-distance assignment with pairs further apart than
//     kMatchDistanceM forbidden. The matcher knows nothing about which track
//     "should" be which target.
// From the matches:
//   * position RMSE per target, over covered, matched instants at least
//     kRmseBurnInS after the target first enters coverage (start-up excluded);
//   * missed target-instants: covered targets unmatched at least kMissBurnInS
//     after they first enter coverage (time for 3 detections plus
//     confirmation);
//   * ID switches: a target matched to a different track than the last time
//     it was matched;
//   * false tracks: reported tracks matched to any target (covered or not) in
//     fewer than half of the instants they were reported (a track that
//     lingers a few seconds after its target leaves coverage is not false).
// kMatchDistanceM = 100 m: about 5x the radar's raw position error at the
// 8 km edge of coverage (sqrt(10^2 + (8000 * 0.0035)^2) ~ 30 m) once
// filtered, yet below the 120 m lane spacing of close_pass, so a track cannot
// be credited to the wrong lane.
//
// Thresholds: the raw radar error is 15-30 m across these geometries and the
// M1 filter averages it to ~0.5-0.7x on a single target; fusing the camera
// lowers it further. The bounds below are the measured baselines plus margin
// (recorded with the tests at the end of this file), so seed-level noise does
// not flake while a broken component fails; see the sensitivity checks noted
// there.

#include <gtest/gtest.h>

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "sifted/association/assignment.hpp"
#include "sifted/core/engine.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted {
namespace {

constexpr double kMatchDistanceM = 100.0;
constexpr double kRmseBurnInS = 10.0;
constexpr double kMissBurnInS = 5.0;

struct TruthRow {
  std::int64_t time_ns = 0;
  double x = 0.0;
  double y = 0.0;
  int target = 0;
  bool covered = true;  // inside the coverage of a sensor that can start tracks
};

struct Scenario {
  SensorTable sensors;
  std::vector<Measurement> measurements;  // decoded frames, in send order
  std::vector<TruthRow> truth;            // sorted by time, then target
};

std::string read_file(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  EXPECT_TRUE(f.good()) << "cannot open " << path;
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::vector<std::uint8_t> from_hex(const std::string& hex) {
  std::vector<std::uint8_t> out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

template <class T>
T parse(std::string_view s) {
  T v{};
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  EXPECT_TRUE(ec == std::errc{} && ptr == s.data() + s.size()) << "bad number: " << s;
  return v;
}

Scenario load(const std::string& name) {
  const std::string dir = std::string(SIFTED_FIXTURE_DIR) + "/scenarios/" + name + "/";
  Scenario sc;
  const auto table = parse_sensor_table(read_file(dir + "sensors.cfg"));
  EXPECT_TRUE(table.ok());
  if (table.ok()) {
    sc.sensors = table.value();
  }
  std::istringstream frames(read_file(dir + "frames.hex"));
  for (std::string line; frames >> line;) {
    const auto bytes = from_hex(line);
    const auto m = wire::decode_measurement(bytes);  // the real C++ decoder
    EXPECT_TRUE(m.ok()) << line;
    if (m.ok()) {
      sc.measurements.push_back(m.value());
    }
  }
  std::istringstream truth(read_file(dir + "truth.csv"));
  std::string line;
  std::getline(truth, line);
  EXPECT_EQ(line, "time_ns,x_m,y_m,vx_mps,vy_mps,target_id,covered");
  while (std::getline(truth, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string_view> f;
    std::string_view rest = line;
    for (std::size_t c; (c = rest.find(',')) != std::string_view::npos; rest.remove_prefix(c + 1)) {
      f.push_back(rest.substr(0, c));
    }
    f.push_back(rest);
    EXPECT_EQ(f.size(), 7U) << line;
    if (f.size() == 7) {
      sc.truth.push_back({parse<std::int64_t>(f[0]), parse<double>(f[1]), parse<double>(f[2]),
                          parse<int>(f[5]), parse<int>(f[6]) != 0});
    }
  }
  return sc;
}

struct Score {
  std::map<int, double> rmse;  // per target
  std::map<int, std::size_t> rmse_n;
  std::size_t eligible = 0;  // target-instants past the miss burn-in
  std::size_t missed = 0;
  std::size_t id_switches = 0;
  std::size_t false_tracks = 0;
  std::size_t reported_tracks = 0;  // distinct ids ever reported
  std::uint64_t tracks_started = 0;
  std::uint64_t track_limit = 0;
  std::size_t bearings = 0;             // bearing-only measurements offered
  std::size_t bearings_associated = 0;  // ... that updated a track
  double mean_rmse = 0.0;               // over targets
  std::vector<TrackEstimate> trace;     // every reported estimate, for determinism
};

Score run(const Scenario& sc, const EngineConfig& cfg = {}) {
  Score s;
  Engine engine(sc.sensors, cfg);
  std::map<int, std::int64_t> first_covered;
  std::map<int, TrackId> last_match;
  std::map<TrackId, std::pair<std::size_t, std::size_t>> track_counts;  // reported, matched
  std::map<int, double> sq;
  std::vector<MeasurementResult> results;

  std::size_t next = 0;
  std::size_t row = 0;
  while (row < sc.truth.size()) {
    const std::int64_t t = sc.truth[row].time_ns;
    std::size_t end = next;
    while (end < sc.measurements.size() && sc.measurements[end].time.ns <= t) {
      ++end;
    }
    engine.on_scan(std::span<const Measurement>(sc.measurements).subspan(next, end - next),
                   results);
    next = end;

    std::vector<TruthRow> now;
    while (row < sc.truth.size() && sc.truth[row].time_ns == t) {
      now.push_back(sc.truth[row]);
      ++row;
    }
    std::vector<TrackEstimate> reported;
    for (TrackEstimate& e : engine.tracks_at(Timestamp{t})) {
      if (e.status == association::TrackStatus::Confirmed ||
          e.status == association::TrackStatus::Coasting) {
        reported.push_back(std::move(e));
      }
    }

    auto distance = [&](std::size_t i, std::size_t j) {
      return std::hypot(reported[j].state(0) - now[i].x, reported[j].state(1) - now[i].y);
    };
    std::vector<int> match(now.size(), association::kUnassigned);
    std::vector<char> taken(reported.size(), 0);
    for (std::size_t i = 0; i < now.size(); ++i) {  // CLEAR-MOT: keep valid correspondences
      const auto it = last_match.find(now[i].target);
      if (it == last_match.end()) {
        continue;
      }
      for (std::size_t j = 0; j < reported.size(); ++j) {
        if (reported[j].id == it->second && taken[j] == 0 && distance(i, j) <= kMatchDistanceM) {
          match[i] = static_cast<int>(j);
          taken[j] = 1;
        }
      }
    }
    std::vector<std::size_t> free_rows;
    std::vector<std::size_t> free_cols;
    for (std::size_t i = 0; i < now.size(); ++i) {
      if (match[i] == association::kUnassigned) {
        free_rows.push_back(i);
      }
    }
    for (std::size_t j = 0; j < reported.size(); ++j) {
      if (taken[j] == 0) {
        free_cols.push_back(j);
      }
    }
    std::vector<double> cost(free_rows.size() * free_cols.size(), association::kForbidden);
    for (std::size_t a = 0; a < free_rows.size(); ++a) {
      for (std::size_t b = 0; b < free_cols.size(); ++b) {
        const double d = distance(free_rows[a], free_cols[b]);
        if (d <= kMatchDistanceM) {
          cost[a * free_cols.size() + b] = d;
        }
      }
    }
    const std::vector<int> rest =
        association::solve_assignment(cost, free_rows.size(), free_cols.size(), kMatchDistanceM);
    for (std::size_t a = 0; a < free_rows.size(); ++a) {
      if (rest[a] != association::kUnassigned) {
        match[free_rows[a]] = static_cast<int>(free_cols[static_cast<std::size_t>(rest[a])]);
      }
    }

    std::vector<char> matched_track(reported.size(), 0);
    for (std::size_t i = 0; i < now.size(); ++i) {
      const int tgt = now[i].target;
      if (match[i] != association::kUnassigned) {
        const TrackEstimate& e = reported[static_cast<std::size_t>(match[i])];
        matched_track[static_cast<std::size_t>(match[i])] = 1;
        if (auto it = last_match.find(tgt); it != last_match.end() && it->second != e.id) {
          ++s.id_switches;
        }
        last_match[tgt] = e.id;
      }
      if (!now[i].covered) {
        continue;  // out of coverage: may be matched, never missed or scored
      }
      first_covered.emplace(tgt, t);
      const double age = static_cast<double>(t - first_covered[tgt]) * 1e-9;
      if (age >= kMissBurnInS) {
        ++s.eligible;
        s.missed += match[i] == association::kUnassigned ? 1U : 0U;
      }
      if (match[i] != association::kUnassigned && age >= kRmseBurnInS) {
        const TrackEstimate& e = reported[static_cast<std::size_t>(match[i])];
        sq[tgt] += std::pow(e.state(0) - now[i].x, 2) + std::pow(e.state(1) - now[i].y, 2);
        ++s.rmse_n[tgt];
      }
    }
    for (std::size_t j = 0; j < reported.size(); ++j) {
      auto& c = track_counts[reported[j].id];
      ++c.first;
      c.second += matched_track[j] != 0 ? 1U : 0U;
      s.trace.push_back(reported[j]);
    }
  }
  for (const auto& [tgt, sum] : sq) {
    s.rmse[tgt] = std::sqrt(sum / static_cast<double>(s.rmse_n[tgt]));
    s.mean_rmse += s.rmse[tgt] / static_cast<double>(sq.size());
  }
  for (std::size_t i = 0; i < results.size() && i < sc.measurements.size(); ++i) {
    if (sc.measurements[i].kind == SensorKind::BearingOnly) {
      ++s.bearings;
      s.bearings_associated += results[i].outcome == MeasurementOutcome::Updated ? 1U : 0U;
    }
  }
  for (const auto& [id, c] : track_counts) {
    if (2 * c.second < c.first) {
      ++s.false_tracks;
    }
  }
  s.reported_tracks = track_counts.size();
  s.tracks_started = engine.tracks_started();
  s.track_limit = engine.count(MeasurementOutcome::TrackLimit);
  return s;
}

struct Bounds {
  double max_rmse_m;        // every target's position RMSE
  double max_missed_ratio;  // missed / eligible target-instants
  std::size_t max_id_switches;
  std::size_t max_false_tracks;
  double min_bearing_association;  // share of camera bearings that updated a track
};

// Fusion must pay: the same frames with every bearing-only measurement
// removed give a worse mean RMSE. Measured ratios are 0.76-0.80; 0.9 leaves
// margin while failing if camera updates stop helping (e.g. a wrong Jacobian).
constexpr double kMaxFusedToRadarOnlyRmse = 0.9;

Scenario without_bearings(Scenario sc) {
  std::erase_if(sc.measurements,
                [](const Measurement& m) { return m.kind == SensorKind::BearingOnly; });
  return sc;
}

void check(const std::string& name, const Bounds& b, std::size_t expected_targets) {
  const Scenario sc = load(name);
  EXPECT_FALSE(sc.measurements.empty());
  const Score s = run(sc);
  const Score radar_only = run(without_bearings(sc));
  const double bearing_share =
      s.bearings ? static_cast<double>(s.bearings_associated) / static_cast<double>(s.bearings)
                 : 0.0;
  std::printf(
      "[ scenario ] %s: targets %zu, missed %zu/%zu (%.3f), id switches %zu, false tracks "
      "%zu, reported tracks %zu, started %llu, bearings associated %zu/%zu (%.3f), mean "
      "RMSE %.2f m vs radar-only %.2f m (%.3f)\n",
      name.c_str(), s.rmse.size(), s.missed, s.eligible,
      s.eligible ? static_cast<double>(s.missed) / static_cast<double>(s.eligible) : 0.0,
      s.id_switches, s.false_tracks, s.reported_tracks,
      static_cast<unsigned long long>(s.tracks_started), s.bearings_associated, s.bearings,
      bearing_share, s.mean_rmse, radar_only.mean_rmse, s.mean_rmse / radar_only.mean_rmse);
  EXPECT_GE(bearing_share, b.min_bearing_association);
  EXPECT_LT(s.mean_rmse, kMaxFusedToRadarOnlyRmse * radar_only.mean_rmse);
  for (const auto& [tgt, r] : s.rmse) {
    std::printf("[ scenario ]   target %d: RMSE %.2f m (n=%zu)\n", tgt, r, s.rmse_n.at(tgt));
    EXPECT_LT(r, b.max_rmse_m) << "target " << tgt;
  }
  EXPECT_EQ(s.rmse.size(), expected_targets) << "every target must be tracked";
  EXPECT_GT(s.eligible, 0U);
  EXPECT_LE(static_cast<double>(s.missed), b.max_missed_ratio * static_cast<double>(s.eligible));
  EXPECT_LE(s.id_switches, b.max_id_switches);
  EXPECT_LE(s.false_tracks, b.max_false_tracks);
  EXPECT_EQ(s.track_limit, 0U);

  // Same input, same bits (ADR 0002).
  const Score again = run(sc);
  EXPECT_EQ(again.trace.size(), s.trace.size());
  for (std::size_t i = 0; i < s.trace.size() && i < again.trace.size(); ++i) {
    ASSERT_EQ(again.trace[i].id, s.trace[i].id);
    ASSERT_TRUE(again.trace[i].state == s.trace[i].state) << "estimate " << i;
    ASSERT_TRUE(again.trace[i].covariance == s.trace[i].covariance) << "estimate " << i;
  }
}

}  // namespace

// Baselines when the bounds were set (GCC 13, default EngineConfig; the
// numbers each test prints):
//
//   scenario       per-target RMSE [m]          missed  IDsw  false  bearings   fused/radar-only
//   crossing_pair  8.39 8.63                    0/172   0     0      338/338    8.51/11.18 (0.76)
//   close_pass     10.49 7.64 12.20             0/258   0     0      425/425   10.11/13.33 (0.76)
//   enter_leave    7.78 9.98 9.62 8.24          0/302   0     0      577/666    8.91/11.10 (0.80)
//
// RMSE bounds sit ~35% above the worst target, like the M1 test. Bearings:
// in crossing_pair and close_pass every bearing comes from a target that is
// already held by a track, so (almost) all must associate; in enter_leave the
// camera also sees targets outside radar coverage, which cannot be tracked.
//
// Sensitivity checks run against these bounds (each broken build fails):
//   gating disabled          28-69% missed, 6-10 ID switches, 3-6 false tracks
//   camera Jacobian sign     18-48% missed, 10-23 ID switches, RMSE 33-58 m
//   camera bearings ignored  0 associated bearings; fused/radar-only = 1.0
//   camera bearing unwrapped crossing_pair loses 2 of 338 bearings at the
//                            seam (0.994 < 0.995). Weak by nature: the gate
//                            rejects the 2 pi innovations, so the damage is
//                            only lost updates. The radar seam is covered
//                            hard by ScenarioRegression.StraightCrossing and
//                            the camera wrap by BearingOnly.* unit tests.
//   manoeuvre detection off  ScenarioRegression.Turn splits its track.
//   greedy instead of GNN    no change: these scenarios never present an
//                            ambiguity greedy gets wrong; the difference is
//                            covered by Assignment.* and Engine.ScanIsAssignedJointly.
TEST(MultiTargetRegression, CrossingPair) { check("crossing_pair", {12.0, 0.02, 0, 0, 0.995}, 2); }

TEST(MultiTargetRegression, ClosePass) { check("close_pass", {16.0, 0.02, 0, 0, 0.99}, 3); }

TEST(MultiTargetRegression, EnterLeave) { check("enter_leave", {14.0, 0.05, 0, 0, 0.8}, 4); }

}  // namespace sifted
