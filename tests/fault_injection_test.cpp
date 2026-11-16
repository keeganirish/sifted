// Sensor and input fault injection (M3): what the system does when a sensor
// restarts, drops out, has a bad clock, is biased or frozen, or floods the
// track table with clutter. Each test checks something meaningful -- bounded
// resources, counters, recovery, determinism -- against measured baselines
// (printed; recorded in docs/threat-model.md, "Sensor and input faults").
// Network-level floods and malformed bursts against the live receive path
// are in udp_flood_test.cpp.

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "faults.hpp"
#include "scenario_support.hpp"
#include "sifted/net/ingest.hpp"

namespace sifted {
namespace {

using testing::PipelineRun;
using testing::Scenario;
using testing::Score;
namespace faults = testing::faults;

constexpr std::int64_t kS = 1'000'000'000;

const std::vector<std::string>& scenario_names() {
  static const std::vector<std::string> n = {"crossing_pair", "close_pass", "enter_leave"};
  return n;
}

SensorId first_of_kind(const SensorTable& t, SensorKind k) {
  for (const SensorConfig& s : t.sensors()) {
    if (s.kind == k) {
      return s.id;
    }
  }
  ADD_FAILURE() << "no sensor of the requested kind";
  return 0;
}

std::int64_t start_of(const Scenario& sc) { return sc.truth.front().time_ns; }

PipelineRun through_pipeline(const Scenario& sc, const std::vector<net::SentFrame>& sent,
                             const net::PipelineConfig& cfg = {}) {
  return testing::run_pipeline(sc, testing::clean_arrivals(sent), cfg);
}

void print(const char* what, const std::string& name, const Score& s) {
  std::printf(
      "[ fault ] %-28s %-13s max RMSE %6.2f mean %6.2f missed %3zu/%zu idsw %zu false %zu "
      "ooo %llu started %llu\n",
      what, name.c_str(), s.max_rmse, s.mean_rmse, s.missed, s.eligible, s.id_switches,
      s.false_tracks, static_cast<unsigned long long>(s.out_of_order),
      static_cast<unsigned long long>(s.tracks_started));
}

// ---- restarts --------------------------------------------------------------

TEST(Faults, SensorRestartIsTransparent) {
  // A restarted sensor's seq numbers start again from 0. Duplicate
  // suppression keys on (seq, time) (ADR 0006), so nothing is mistaken for a
  // repeat: the track picture is bit-for-bit the one without the restart.
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const SensorId radar = first_of_kind(sc.sensors, SensorKind::RangeBearing);
    const SensorId camera = first_of_kind(sc.sensors, SensorKind::BearingOnly);
    auto ms = faults::restart(sc.measurements, radar, start_of(sc) + 30 * kS);
    ms = faults::restart(ms, camera, start_of(sc) + 45 * kS);
    ms = faults::restart(ms, radar, start_of(sc) + 46 * kS);  // twice, soon after
    const PipelineRun base = through_pipeline(sc, testing::sent_frames(sc.measurements));
    const PipelineRun faulted = through_pipeline(sc, testing::sent_frames(ms));
    EXPECT_EQ(faulted.duplicates, 0U) << name;
    EXPECT_EQ(faulted.score.measurements, base.score.measurements);
    ASSERT_EQ(faulted.score.trace.size(), base.score.trace.size()) << name;
    for (std::size_t i = 0; i < base.score.trace.size(); ++i) {
      ASSERT_EQ(faulted.score.trace[i].id, base.score.trace[i].id);
      ASSERT_TRUE(faulted.score.trace[i].state == base.score.trace[i].state) << name << " " << i;
    }
  }
}

// ---- dropouts ----------------------------------------------------------------

TEST(Faults, RadarDropoutCoastsAndRecovers) {
  // The only track-holding sensor goes silent for 6 s; camera bearings keep
  // refining the tracks but cannot hold them (lifecycle: bearing-only updates
  // neither confirm nor keep a track out of coasting).
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const SensorId radar = first_of_kind(sc.sensors, SensorKind::RangeBearing);
    const std::int64_t a = start_of(sc) + 30 * kS;
    const std::int64_t b = a + 6 * kS;
    const Score s = testing::run_engine(sc, faults::dropout(sc.measurements, radar, a, b));
    const testing::Recovery rec = testing::recovery_after(s, a, b);
    print("radar dropout 6 s", name, s);
    std::printf("[ fault ]   recovered after %.1f s, survived %zu/%zu\n", rec.recovery_s,
                rec.survived, rec.targets);
    EXPECT_GE(rec.recovery_s, 0.0) << name;
    EXPECT_LE(rec.recovery_s, 4.0) << name;
    EXPECT_LE(s.false_tracks, 0U) << name;
  }
}

TEST(Faults, IntermittentCameraDegradesGracefully) {
  // The camera is off for random 2 s periods (40% of them). Fusion gains
  // shrink, nothing breaks.
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const SensorId camera = first_of_kind(sc.sensors, SensorKind::BearingOnly);
    const Score base = testing::run_engine(sc);
    const Score s =
        testing::run_engine(sc, faults::intermittent(sc.measurements, camera, 2 * kS, 0.4, 17));
    print("intermittent camera", name, s);
    EXPECT_EQ(s.id_switches, base.id_switches) << name;
    EXPECT_EQ(s.false_tracks, base.false_tracks) << name;
    EXPECT_LE(s.missed, base.missed + 2) << name;
    EXPECT_LT(s.max_rmse, 1.35 * base.max_rmse + 2.0) << name;
  }
}

// ---- clock faults --------------------------------------------------------------

struct ClockCase {
  const char* what;
  bool camera;  // which sensor's clock is wrong
  std::int64_t offset_ns;
  double drift_ppm;
};

TEST(Faults, ClockFaultsAndTheirMitigation) {
  // The engine trusts sensor time. A sensor whose clock runs AHEAD moves the
  // engine clock forward, so every other sensor's on-time data becomes
  // OutOfOrder: one bad clock silences the rest. A sensor whose clock runs
  // BEHIND only loses its own data. Two recorded, deterministic stages
  // mitigate this (ADRs 0009, 0010): the reorder window absorbs a lead
  // smaller than the window (frames are released in measurement-time order),
  // and the clock gate (max_time_ahead_s, comparing measurement time with
  // receive time) rejects a larger lead before it can move the clock. With
  // the gate no wider than the window, no lead gets through unabsorbed.
  const std::vector<ClockCase> cases = {
      {"camera +2 s", true, 2 * kS, 0.0},
      {"camera -2 s", true, -2 * kS, 0.0},
      {"camera drift +5000 ppm", true, 0, 5000.0},
      {"radar +2 s", false, 2 * kS, 0.0},
  };
  net::PipelineConfig mitigated;
  mitigated.reorder_window_s = 0.5;
  mitigated.max_time_ahead_s = 0.5;
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const SensorId radar = first_of_kind(sc.sensors, SensorKind::RangeBearing);
    const SensorId camera = first_of_kind(sc.sensors, SensorKind::BearingOnly);
    const PipelineRun base = through_pipeline(sc, testing::sent_frames(sc.measurements), mitigated);
    for (const ClockCase& c : cases) {
      const SensorId bad = c.camera ? camera : radar;
      const auto stamped = faults::clock_fault(sc.measurements, bad, c.offset_ns, c.drift_ppm);
      const auto sent = faults::sent_with_true_times(sc.measurements, stamped);
      const PipelineRun open = through_pipeline(sc, sent);
      const PipelineRun fixed = through_pipeline(sc, sent, mitigated);
      print((std::string(c.what) + ", M2 policy").c_str(), name, open.score);
      print((std::string(c.what) + ", mitigated").c_str(), name, fixed.score);
      std::printf("[ fault ]   clock gate rejected %llu, reorder late %llu\n",
                  static_cast<unsigned long long>(fixed.future_rejected),
                  static_cast<unsigned long long>(fixed.reorder_late));

      std::size_t from_bad = 0;
      std::size_t too_far_ahead = 0;  // what the gate must refuse: lead > max_time_ahead_s
      for (std::size_t i = 0; i < sc.measurements.size(); ++i) {
        from_bad += sc.measurements[i].sensor_id == bad ? 1U : 0U;
        too_far_ahead +=
            static_cast<double>(stamped[i].time.ns - sc.measurements[i].time.ns) * 1e-9 >
                    mitigated.max_time_ahead_s
                ? 1U
                : 0U;
      }
      EXPECT_EQ(fixed.future_rejected, too_far_ahead) << name << " " << c.what;
      if (c.offset_ns > 0) {
        // Unmitigated, the fast clock costs the other sensors' data...
        EXPECT_GT(open.score.out_of_order, from_bad / 2) << name << " " << c.what;
        // ...mitigated, exactly the bad sensor's frames are refused and the
        // rest is processed in order.
        EXPECT_EQ(fixed.future_rejected, from_bad) << name << " " << c.what;
        EXPECT_EQ(fixed.score.out_of_order, 0U) << name << " " << c.what;
      }
      if (c.camera && c.offset_ns > 0) {
        // Losing the camera leaves radar-only tracking: identity intact.
        EXPECT_EQ(fixed.score.id_switches, base.score.id_switches) << name;
        EXPECT_LE(fixed.score.missed, base.score.missed + 2) << name;
      }
      if (!c.camera && c.offset_ns > 0) {
        // Losing the only track-starting sensor leaves no tracks at all: the
        // gate trades availability for integrity (no picture rather than a
        // wrong one).
        EXPECT_EQ(fixed.score.tracks_started, 0U) << name;
      }
      if (c.offset_ns < 0) {
        // A slow clock only hurts its own sensor (its data arrives "late").
        EXPECT_GT(open.score.out_of_order, from_bad / 2) << name;
        EXPECT_EQ(fixed.score.id_switches, base.score.id_switches) << name;
        EXPECT_LE(fixed.score.missed, base.score.missed + 2) << name;
      }
      if (c.drift_ppm != 0.0) {
        // 5000 ppm drifts 0.45 s over 90 s (0.6 s over enter_leave's 120 s).
        // That soon exceeds the 0.25 s between camera and radar frames, so
        // unmitigated it costs radar scans; mitigated, the lead is absorbed
        // by the reorder window while it is small and refused by the gate
        // (checked above) once it is not.
        EXPECT_GT(open.score.out_of_order, 20U) << name;
        EXPECT_LE(fixed.score.out_of_order, from_bad / 20) << name;
        EXPECT_EQ(fixed.score.id_switches, base.score.id_switches) << name;
        EXPECT_LE(fixed.score.missed, base.score.missed + 2) << name;
      }
    }
  }
}

// ---- biased and stuck sensors ------------------------------------------------------

TEST(Faults, BiasedRadarShiftsThePictureAndConflictsWithTheCamera) {
  // A constant 50 m range bias cannot be detected from one radar's data
  // alone (it is a registration problem; sensor bias estimation is out of
  // scope). Alone, the biased radar must give a consistently shifted but
  // intact picture. Fused with an unbiased camera at another site, the two
  // disagree; the filter is pulled between them and identity can break. Both
  // are recorded so a regression in either direction shows.
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const SensorId radar = first_of_kind(sc.sensors, SensorKind::RangeBearing);
    std::vector<Measurement> radar_only;
    for (const Measurement& m : sc.measurements) {
      if (m.sensor_id == radar) {
        radar_only.push_back(m);
      }
    }
    const Score base = testing::run_engine(sc, radar_only);
    const Score alone = testing::run_engine(sc, faults::bias(radar_only, radar, 50.0));
    const Score fused = testing::run_engine(sc, faults::bias(sc.measurements, radar, 50.0));
    print("radar bias 50 m, radar only", name, alone);
    print("radar bias 50 m, with camera", name, fused);
    EXPECT_EQ(alone.id_switches, base.id_switches) << name;
    EXPECT_EQ(alone.false_tracks, base.false_tracks) << name;
    EXPECT_LE(alone.missed, base.missed + 2) << name;
    EXPECT_GT(alone.mean_rmse, base.mean_rmse + 25.0) << name << ": the bias must be visible";
    EXPECT_LT(alone.max_rmse, 50.0 + base.max_rmse) << name;
    // Fused: every target still tracked, error bounded by the bias; identity
    // errors measured 1-6 (recorded, bounded loosely).
    EXPECT_EQ(fused.rmse.size(), base.rmse.size()) << name;
    EXPECT_LT(fused.max_rmse, 50.0 + base.max_rmse) << name;
    EXPECT_LE(fused.id_switches, 10U) << name;
  }
}

TEST(Faults, StuckRadarRecoversWhenItUnfreezes) {
  // The radar repeats one scan for 8 s. Its detections stop matching the
  // moving targets, the tracks lose their position updates, and once the
  // radar recovers, the targets are tracked again within a few revisits.
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const SensorId radar = first_of_kind(sc.sensors, SensorKind::RangeBearing);
    const std::int64_t a = start_of(sc) + 30 * kS;
    const std::int64_t b = a + 8 * kS;
    EngineConfig cfg;
    cfg.max_tracks = 64;
    const auto ms = faults::stuck(sc.measurements, radar, a, b);
    const Score s = testing::run_engine(sc, ms, cfg);
    const testing::Recovery rec = testing::recovery_after(s, a, b);
    std::size_t frozen = 0;  // detections in the scan the radar repeats
    std::int64_t last = INT64_MIN;
    for (const Measurement& m : sc.measurements) {
      if (m.sensor_id == radar && m.time.ns < a) {
        frozen = m.time.ns == last ? frozen + 1 : 1;
        last = m.time.ns;
      }
    }
    print("stuck radar 8 s", name, s);
    std::printf("[ fault ]   recovered after %.1f s, survived %zu/%zu, frozen scan %zu\n",
                rec.recovery_s, rec.survived, rec.targets, frozen);
    EXPECT_GE(rec.recovery_s, 0.0) << name;
    EXPECT_LE(rec.recovery_s, 5.0) << name;
    EXPECT_EQ(s.track_limit, 0U);
    // The repeated detections look like stationary targets and are confirmed
    // as such: at most one false track per frozen detection, gone after the
    // radar recovers (they are not reported at the end).
    EXPECT_LE(s.false_tracks, frozen) << name;
  }
}

// ---- track-cap pressure --------------------------------------------------------

TEST(Faults, ClutterFloodRespectsTheTrackCapAndKeepsEstablishedTracks) {
  // 300 extra false alarms per radar scan from t = 20 s, against a cap of 64
  // live tracks. Every scan, the live-track count, the trace buffers and the
  // measurement log must stay within their bounds; established tracks are
  // associated before tentative ones, so the real targets must still be held.
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const SensorId radar = first_of_kind(sc.sensors, SensorKind::RangeBearing);
    std::vector<Measurement> ms;
    const std::int64_t flood_from = start_of(sc) + 20 * kS;
    std::vector<Measurement> before;
    std::vector<Measurement> after;
    for (const Measurement& m : sc.measurements) {
      (m.time.ns < flood_from ? before : after).push_back(m);
    }
    ms = before;
    for (const Measurement& m : faults::clutter(after, radar, 300, 8000.0, 23)) {
      ms.push_back(m);
    }
    EngineConfig cfg;
    cfg.max_tracks = 64;
    Engine engine(sc.sensors, cfg);
    std::vector<MeasurementResult> results;
    std::size_t begin = 0;
    while (begin < ms.size()) {
      std::size_t end = begin + 1;
      while (end < ms.size() && ms[end].time == ms[begin].time &&
             ms[end].sensor_id == ms[begin].sensor_id) {
        ++end;
      }
      engine.on_scan(std::span<const Measurement>(ms).subspan(begin, end - begin), results);
      begin = end;
      ASSERT_LE(engine.store().size(), cfg.max_tracks);
      for (const auto& r : engine.store().live()) {
        ASSERT_LE(r.recent.size(), cfg.trace.events_per_track);
        ASSERT_LE(r.sensors.size(), cfg.trace.sensors_per_track);
      }
      ASSERT_LE(engine.store().dropped().size(), cfg.trace.dropped_tracks_kept);
      ASSERT_LE(engine.store().measurements().size(), cfg.trace.measurement_log);
    }
    EXPECT_EQ(results.size(), ms.size()) << "one outcome per measurement";
    EXPECT_GT(engine.count(MeasurementOutcome::TrackLimit), 1000U) << name;

    const Score s = testing::run_engine(sc, ms, cfg);
    print("clutter 300/scan, cap 64", name, s);
    // Targets confirmed before the flood keep their tracks; a target that
    // only enters coverage during the flood may be unable to start one while
    // the table is full (enter_leave's late arrivals), which is the cost of a
    // hard cap and is visible in the TrackLimit counter.
    if (name != "enter_leave") {
      EXPECT_LE(s.missed_ratio(), 0.05) << name;
      EXPECT_EQ(s.id_switches, 0U) << name;
    }
  }
}

// ---- determinism ------------------------------------------------------------

TEST(Faults, FaultedRunsAreDeterministic) {
  const Scenario sc = testing::load_scenario("enter_leave");
  const SensorId radar = first_of_kind(sc.sensors, SensorKind::RangeBearing);
  const SensorId camera = first_of_kind(sc.sensors, SensorKind::BearingOnly);
  auto ms = faults::clock_fault(sc.measurements, camera, 3 * kS);
  ms = faults::stuck(ms, radar, start_of(sc) + 40 * kS, start_of(sc) + 45 * kS);
  ms = faults::clutter(ms, radar, 50, 6000.0, 5);
  net::PipelineConfig cfg;
  cfg.max_time_ahead_s = 1.0;
  cfg.reorder_window_s = 0.5;
  cfg.engine.max_tracks = 128;
  std::vector<Measurement> truth_times = ms;  // send at the stamped time except the camera's
  for (std::size_t i = 0; i < ms.size(); ++i) {
    if (ms[i].sensor_id == camera) {
      truth_times[i].time.ns -= 3 * kS;
    }
  }
  const auto sent = faults::sent_with_true_times(truth_times, ms);
  std::vector<net::SentFrame> ordered = sent;
  std::stable_sort(
      ordered.begin(), ordered.end(),
      [](const net::SentFrame& a, const net::SentFrame& b) { return a.send < b.send; });
  const PipelineRun a = through_pipeline(sc, ordered, cfg);
  const PipelineRun b = through_pipeline(sc, ordered, cfg);
  EXPECT_FALSE(a.output.empty());
  EXPECT_EQ(a.output, b.output);
}

}  // namespace
}  // namespace sifted
