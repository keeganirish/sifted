// Association and track lifecycle: chi-square gates, the optimal assignment
// solver, deterministic tie-breaking, two-pass GNN, and the
// tentative -> confirmed -> coasting -> dropped lifecycle.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "sifted/association/assignment.hpp"
#include "sifted/association/gating.hpp"
#include "sifted/core/engine.hpp"
#include "sifted/estimation/geodetic.hpp"
#include "sifted/estimation/range_bearing.hpp"

namespace sifted {
namespace {

using association::TrackStatus;

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr SensorId kRadar = 1;
constexpr SensorId kCamera = 2;
const SensorPose kRadarPose{0.0, 0.0, 0.0};
const SensorPose kCameraPose{-2000.0, 0.0, 0.0};

SensorTable table() {
  SensorTable t;
  EXPECT_FALSE(t.add({kRadar, SensorKind::RangeBearing, kRadarPose}));
  EXPECT_FALSE(t.add({kCamera, SensorKind::BearingOnly, kCameraPose}));
  return t;
}

Measurement radar_at(double x, double y, std::int64_t t_ns, std::uint32_t seq) {
  estimation::StateVector s = estimation::StateVector::Zero();
  s(0) = x;
  s(1) = y;
  const estimation::RangeBearing z = estimation::predict_range_bearing(s, kRadarPose);
  Measurement m;
  m.sensor_id = kRadar;
  m.kind = SensorKind::RangeBearing;
  m.seq = seq;
  m.time = Timestamp{t_ns};
  m.z = {z.range_m, z.bearing_rad, 0.0};
  m.sigma = {10.0, 0.0035, 0.0};
  return m;
}

Measurement camera_at(double x, double y, std::int64_t t_ns, std::uint32_t seq) {
  Measurement m;
  m.sensor_id = kCamera;
  m.kind = SensorKind::BearingOnly;
  m.seq = seq;
  m.time = Timestamp{t_ns};
  m.z = {estimation::wrap_angle(std::atan2(y - kCameraPose.y_m, x - kCameraPose.x_m) -
                                kCameraPose.yaw_rad),
         0.0, 0.0};
  m.sigma = {0.002, 0.0, 0.0};
  return m;
}

// Feeds `n` noiseless 1 Hz radar hits of a target moving at (vx, vy) from (x0, y0).
std::uint32_t feed_line(Engine& e, double x0, double y0, double vx, double vy, int n,
                        std::uint32_t seq0 = 0, std::int64_t t0 = 0) {
  std::uint32_t seq = seq0;
  for (int k = 0; k < n; ++k) {
    const double s = static_cast<double>(k);
    e.on_measurement(radar_at(x0 + vx * s, y0 + vy * s, t0 + k * kSecond, seq++));
  }
  return seq;
}

// Total cost of an assignment (unassigned rows cost `u`).
double total(const std::vector<double>& cost, std::size_t cols, const std::vector<int>& a,
             double u) {
  double sum = 0.0;
  for (std::size_t r = 0; r < a.size(); ++r) {
    sum += a[r] == association::kUnassigned ? u : cost[r * cols + static_cast<std::size_t>(a[r])];
  }
  return sum;
}

// Exhaustive minimum over all partial injections rows -> cols.
double brute_force(const std::vector<double>& cost, std::size_t rows, std::size_t cols, double u,
                   std::size_t r = 0, std::vector<char>* used = nullptr) {
  std::vector<char> local(cols, 0);
  if (used == nullptr) {
    used = &local;
  }
  if (r == rows) {
    return 0.0;
  }
  double best = u + brute_force(cost, rows, cols, u, r + 1, used);
  for (std::size_t c = 0; c < cols; ++c) {
    const double v = cost[r * cols + c];
    if ((*used)[c] != 0 || v == association::kForbidden) {
      continue;
    }
    (*used)[c] = 1;
    best = std::min(best, v + brute_force(cost, rows, cols, u, r + 1, used));
    (*used)[c] = 0;
  }
  return best;
}

}  // namespace

// ---- gating ----------------------------------------------------------------

TEST(ChiSquare, QuantilesMatchTables) {
  // Reference values from standard chi-square tables.
  EXPECT_NEAR(*association::chi_square_quantile(1, 0.99), 6.6349, 1e-4);
  EXPECT_NEAR(*association::chi_square_quantile(2, 0.99), 9.2103, 1e-4);
  EXPECT_NEAR(*association::chi_square_quantile(1, 0.999), 10.8276, 1e-4);
  EXPECT_NEAR(*association::chi_square_quantile(2, 0.999), 13.8155, 1e-4);
  EXPECT_NEAR(*association::chi_square_quantile(3, 0.95), 7.8147, 1e-4);
  EXPECT_NEAR(*association::chi_square_quantile(18, 0.99), 34.8053, 1e-4);
  EXPECT_NEAR(association::chi_square_cdf(2, *association::chi_square_quantile(2, 0.5)), 0.5,
              1e-12);
  EXPECT_FALSE(association::chi_square_quantile(0, 0.9).has_value());
  EXPECT_FALSE(association::chi_square_quantile(2, 1.0).has_value());
  EXPECT_FALSE(association::chi_square_quantile(2, 0.0).has_value());
}

TEST(Engine, GateThresholdsFollowDimensionAndProbability) {
  Engine e(table());
  EXPECT_NEAR(e.gate(1), 10.8276, 1e-4);
  EXPECT_NEAR(e.gate(2), 13.8155, 1e-4);
}

// ---- assignment solver ------------------------------------------------------

TEST(Assignment, MatchesBruteForceOnRandomProblems) {
  std::uint64_t state = 12345;
  auto next = [&state]() {  // fixed LCG: deterministic test data
    state = state * 6364136223846793005ULL + 1442695040888963407ULL;
    return static_cast<double>(state >> 11) / 9007199254740992.0;
  };
  for (int trial = 0; trial < 300; ++trial) {
    const auto rows = static_cast<std::size_t>(1 + next() * 5);
    const auto cols = static_cast<std::size_t>(1 + next() * 5);
    const double u = 3.0 + next() * 5.0;
    std::vector<double> cost(rows * cols);
    for (double& c : cost) {
      c = next() < 0.3 ? association::kForbidden : next() * 10.0;
    }
    const std::vector<int> a = association::solve_assignment(cost, rows, cols, u);
    ASSERT_EQ(a.size(), rows);
    std::vector<int> seen;
    for (int c : a) {
      if (c != association::kUnassigned) {
        ASSERT_LT(static_cast<std::size_t>(c), cols);
        seen.push_back(c);
      }
    }
    std::sort(seen.begin(), seen.end());
    ASSERT_TRUE(std::adjacent_find(seen.begin(), seen.end()) == seen.end()) << "column reused";
    for (std::size_t r = 0; r < rows; ++r) {
      if (a[r] != association::kUnassigned) {
        ASSERT_NE(cost[r * cols + static_cast<std::size_t>(a[r])], association::kForbidden);
      }
    }
    EXPECT_NEAR(total(cost, cols, a, u), brute_force(cost, rows, cols, u), 1e-9)
        << "trial " << trial;
  }
}

TEST(Assignment, BeatsGreedyNearestNeighbour) {
  // Greedy would give row 0 its nearest column (1, cost 1) and leave row 1
  // with nothing (its only option is column 1). Optimal: 0->0, 1->1.
  const std::vector<double> cost{2.0, 1.0, association::kForbidden, 1.5};
  const auto a = association::solve_assignment(cost, 2, 2, 10.0);
  EXPECT_EQ(a, (std::vector<int>{0, 1}));
}

TEST(Assignment, TiesGoToTheLowestColumn) {
  const std::vector<double> cost{1.0, 1.0, 1.0};
  EXPECT_EQ(association::solve_assignment(cost, 1, 3, 5.0), std::vector<int>{0});
}

TEST(Assignment, LeavesRowsUnassignedWhenCheaper) {
  const std::vector<double> cost{9.0, association::kForbidden};
  EXPECT_EQ(association::solve_assignment(cost, 2, 1, 5.0),
            (std::vector<int>{association::kUnassigned, association::kUnassigned}));
}

TEST(Assignment, ConnectedComponents) {
  // rows 0-1 share col 0; row 2 alone with col 2; row 3 has no edge.
  const auto comps = association::connected_components({{0, 0}, {1, 0}, {1, 1}, {2, 2}}, 4, 3);
  ASSERT_EQ(comps.size(), 2U);
  EXPECT_EQ(comps[0].rows, (std::vector<std::size_t>{0, 1}));
  EXPECT_EQ(comps[0].cols, (std::vector<std::size_t>{0, 1}));
  EXPECT_EQ(comps[1].rows, (std::vector<std::size_t>{2}));
  EXPECT_EQ(comps[1].cols, (std::vector<std::size_t>{2}));
}

// ---- engine association -----------------------------------------------------

TEST(Engine, GatesOutFarMeasurementsAndStartsANewTrack) {
  Engine e(table());
  feed_line(e, 5000.0, 1000.0, -50.0, 0.0, 5);
  ASSERT_EQ(e.tracks().size(), 1U);
  // 2 km away from the predicted position: far outside the gate.
  const MeasurementResult r = e.on_measurement_result(radar_at(4750.0, 3000.0, 5 * kSecond, 99));
  EXPECT_EQ(r.outcome, MeasurementOutcome::Initialized);
  EXPECT_EQ(r.track, 2U);
  EXPECT_EQ(e.tracks().size(), 2U);
}

TEST(Engine, TiesAreBrokenByTrackId) {
  Engine e(table());
  // One scan with two detections at the same point: two identical tracks.
  const std::vector<Measurement> scan{radar_at(3000.0, 0.0, 0, 0), radar_at(3000.0, 0.0, 0, 1)};
  std::vector<MeasurementResult> out;
  e.on_scan(scan, out);
  ASSERT_EQ(out.size(), 2U);
  EXPECT_EQ(out[0].track, 1U);
  EXPECT_EQ(out[1].track, 2U);
  // A later detection exactly between them is equally likely for both.
  const MeasurementResult r = e.on_measurement_result(radar_at(3000.0, 0.0, kSecond, 2));
  EXPECT_EQ(r.outcome, MeasurementOutcome::Updated);
  EXPECT_EQ(r.track, 1U);
}

TEST(Engine, ScanIsAssignedJointly) {
  Engine e(table());
  // Two targets 300 m apart, flying in parallel.
  for (std::uint32_t k = 0; k < 6; ++k) {
    const double s = static_cast<double>(k);
    const std::vector<Measurement> scan{radar_at(4000.0 + 30.0 * s, 0.0, k * kSecond, 2 * k),
                                        radar_at(4000.0 + 30.0 * s, 300.0, k * kSecond, 2 * k + 1)};
    std::vector<MeasurementResult> out;
    e.on_scan(scan, out);
    ASSERT_EQ(out[0].track, 1U) << "scan " << k;
    ASSERT_EQ(out[1].track, 2U) << "scan " << k;
  }
  // The same scan reported in the opposite order changes nothing about who is who.
  const std::vector<Measurement> swapped{radar_at(4180.0, 300.0, 6 * kSecond, 13),
                                         radar_at(4180.0, 0.0, 6 * kSecond, 12)};
  std::vector<MeasurementResult> out;
  e.on_scan(swapped, out);
  EXPECT_EQ(out[0].track, 2U);
  EXPECT_EQ(out[1].track, 1U);
}

TEST(Engine, EstablishedTracksAreServedBeforeTentativeOnes) {
  Engine e(table());
  feed_line(e, 4000.0, 0.0, 0.0, 0.0, 4);  // track 1, confirmed
  ASSERT_EQ(e.track(1)->status, TrackStatus::Confirmed);
  // A tentative track 150 m away (outside track 1's ~75 m gate).
  ASSERT_EQ(e.on_measurement_result(radar_at(4000.0, 150.0, 4 * kSecond, 50)).track, 2U);
  // A detection 50 m from track 1 and 100 m from the (very uncertain)
  // tentative track 2 is inside both gates; it goes to the established track.
  const MeasurementResult r = e.on_measurement_result(radar_at(4000.0, 50.0, 5 * kSecond, 51));
  EXPECT_EQ(r.track, 1U);
  const auto* t2 = e.store().find(2);
  ASSERT_NE(t2, nullptr);
  EXPECT_EQ(t2->totals[static_cast<std::size_t>(store::TraceEventKind::Competed)], 1U);
}

TEST(Engine, DuplicateWithinAScanIsRejected) {
  Engine e(table());
  const std::vector<Measurement> scan{radar_at(3000.0, 0.0, 0, 7), radar_at(3000.0, 0.0, 0, 7)};
  std::vector<MeasurementResult> out;
  e.on_scan(scan, out);
  EXPECT_EQ(out[0].outcome, MeasurementOutcome::Initialized);
  EXPECT_EQ(out[1].outcome, MeasurementOutcome::Duplicate);
}

// ---- bearing-only fusion -----------------------------------------------------

TEST(Engine, BearingOnlyUpdatesButNeverStartsTracks) {
  Engine e(table());
  EXPECT_EQ(e.on_measurement(camera_at(3000.0, 500.0, 0, 0)), MeasurementOutcome::Unassociated);
  EXPECT_TRUE(e.tracks().empty());
  feed_line(e, 3000.0, 500.0, 0.0, 0.0, 3, 0, kSecond);
  const MeasurementResult r = e.on_measurement_result(camera_at(3000.0, 500.0, 4 * kSecond, 1));
  EXPECT_EQ(r.outcome, MeasurementOutcome::Updated);
  EXPECT_EQ(r.track, 1U);
  EXPECT_EQ(e.track(1)->sensors, (std::vector<SensorId>{kRadar, kCamera}));
}

TEST(Engine, BearingOnlyHitsDoNotConfirm) {
  Engine e(table());
  e.on_measurement(radar_at(3000.0, 500.0, 0, 0));
  for (std::uint32_t k = 1; k <= 5; ++k) {
    ASSERT_EQ(e.on_measurement(camera_at(3000.0, 500.0, k * kSecond / 2, k)),
              MeasurementOutcome::Updated);
  }
  EXPECT_EQ(e.track(1)->status, TrackStatus::Tentative);
}

// ---- lifecycle ---------------------------------------------------------------

TEST(Lifecycle, ConfirmsAfterThreeHits) {
  Engine e(table());
  e.on_measurement(radar_at(3000.0, 0.0, 0, 0));
  e.on_measurement(radar_at(3000.0, 0.0, kSecond, 1));
  EXPECT_EQ(e.track(1)->status, TrackStatus::Tentative);
  e.on_measurement(radar_at(3000.0, 0.0, 2 * kSecond, 2));
  EXPECT_EQ(e.track(1)->status, TrackStatus::Confirmed);
}

TEST(Lifecycle, TentativeTrackTimesOut) {
  Engine e(table());
  e.on_measurement(radar_at(3000.0, 0.0, 0, 0));
  // An unrelated detection far away, 4 s later, advances the clock.
  e.on_measurement(radar_at(-3000.0, 0.0, 4 * kSecond, 1));
  EXPECT_FALSE(e.track(1).has_value());
  EXPECT_EQ(e.tracks_dropped(association::DropReason::TentativeTimeout), 1U);
  ASSERT_NE(e.store().find_any(1), nullptr);  // still explainable
}

TEST(Lifecycle, CoastsRecoversAndDrops) {
  Engine e(table());
  std::uint32_t seq = feed_line(e, 3000.0, 0.0, 0.0, 0.0, 4);  // confirmed at t=2, last t=3
  // Far-away detections drive the clock while the target is unobserved.
  e.on_measurement(radar_at(-3000.0, 0.0, 6 * kSecond, seq++));  // 3 s gap > 2.5 s
  EXPECT_EQ(e.track(1)->status, TrackStatus::Coasting);
  e.on_measurement(radar_at(3000.0, 0.0, 7 * kSecond, seq++));
  EXPECT_EQ(e.track(1)->status, TrackStatus::Confirmed);
  e.on_measurement(radar_at(-3000.0, 0.0, 18 * kSecond, seq++));  // 11 s gap > 10 s
  EXPECT_FALSE(e.track(1).has_value());
  EXPECT_EQ(e.tracks_dropped(association::DropReason::Stale), 1U);
}

TEST(Lifecycle, DropsTracksThatBecomeTooUncertain) {
  EngineConfig cfg;
  cfg.lifecycle.max_position_sigma_m = 50.0;
  cfg.lifecycle.tentative_timeout_s = 100.0;
  Engine e(table(), cfg);
  // A single fix with a 100 m/s velocity prior: 1 s later sigma is ~100 m.
  e.on_measurement(radar_at(3000.0, 0.0, 0, 0));
  e.on_measurement(radar_at(-3000.0, 0.0, kSecond, 1));
  EXPECT_FALSE(e.track(1).has_value());
  EXPECT_EQ(e.tracks_dropped(association::DropReason::Uncertainty), 1U);
}

TEST(Lifecycle, TrackCapIsEnforcedAndCounted) {
  EngineConfig cfg;
  cfg.max_tracks = 2;
  Engine e(table(), cfg);
  EXPECT_EQ(e.on_measurement(radar_at(3000.0, 0.0, 0, 0)), MeasurementOutcome::Initialized);
  EXPECT_EQ(e.on_measurement(radar_at(-3000.0, 0.0, 0, 1)), MeasurementOutcome::Initialized);
  EXPECT_EQ(e.on_measurement(radar_at(0.0, 3000.0, 0, 2)), MeasurementOutcome::TrackLimit);
  EXPECT_EQ(e.count(MeasurementOutcome::TrackLimit), 1U);
  EXPECT_EQ(e.tracks().size(), 2U);
}

TEST(Lifecycle, ManoeuvreRaisesProcessNoiseAndIsTraced) {
  Engine e(table());
  std::uint32_t seq = feed_line(e, 4000.0, 0.0, 0.0, 50.0, 10);
  EXPECT_FALSE(e.track(1)->manoeuvring);
  // From t = 10 s, a sustained 4 m/s^2 lateral acceleration (noiseless).
  bool detected = false;
  for (int k = 1; k <= 20 && !detected; ++k) {
    const double s = static_cast<double>(k);
    e.on_measurement(radar_at(4000.0 + 2.0 * s * s, 450.0 + 50.0 * s, (9 + k) * kSecond, seq++));
    ASSERT_TRUE(e.track(1).has_value()) << "track lost at k=" << k;
    detected = e.track(1)->manoeuvring;
  }
  EXPECT_TRUE(detected);
  EXPECT_NE(e.explain(1)->find("manoeuvre start"), std::string::npos);
}

// ---- geodetic (ADS-B) measurements --------------------------------------------

namespace {
Measurement geo_at(double lat, double lon, std::int64_t t_ns, std::uint32_t seq) {
  Measurement m;
  m.sensor_id = 9;
  m.kind = SensorKind::GeoPosition;
  m.seq = seq;
  m.time = Timestamp{t_ns};
  m.z = {lat, lon, 3000.0};
  m.sigma = {50.0 / 111320.0, 50.0 / (111320.0 * std::cos(lat * 3.14159265358979 / 180.0)), 50.0};
  return m;
}
SensorTable geo_table() {
  SensorTable t;
  EXPECT_FALSE(t.add({9, SensorKind::GeoPosition, {0.0, 0.0, 0.0}}));
  return t;
}
}  // namespace

TEST(Engine, GeoPositionNeedsAnOriginAndIsTrackedInTheTangentPlane) {
  Engine no_origin(geo_table());
  EXPECT_EQ(no_origin.on_measurement(geo_at(51.5, 0.0, 0, 0)), MeasurementOutcome::UnsupportedKind);

  EngineConfig cfg;
  cfg.origin = estimation::GeoOrigin{51.4779, -0.0015, 0.0};
  Engine e(geo_table(), cfg);
  // An aircraft flying north at ~200 m/s (0.0018 deg/s of latitude), 1 Hz.
  for (std::uint32_t k = 0; k < 5; ++k) {
    const double lat = 51.5 + 0.0018 * k;
    ASSERT_EQ(e.on_measurement(geo_at(lat, 0.1, k * kSecond, k)),
              k == 0 ? MeasurementOutcome::Initialized : MeasurementOutcome::Updated);
  }
  const auto t = e.track(1);
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->status, TrackStatus::Confirmed);  // geodetic fixes observe position: they confirm
  const Eigen::Vector3d enu = estimation::geodetic_to_enu(*cfg.origin, {51.5072, 0.1, 3000.0});
  EXPECT_NEAR(t->state(0), enu(0), 30.0);
  EXPECT_NEAR(t->state(1), enu(1), 30.0);
  EXPECT_NEAR(t->state(3), 200.0, 15.0);
  // Far beyond the tangent plane's validity range.
  EXPECT_EQ(e.on_measurement(geo_at(-30.0, 120.0, 5 * kSecond, 9)), MeasurementOutcome::OutOfFrame);
}

TEST(Engine, InvalidConfigIsRejected) {
  EngineConfig cfg;
  cfg.association.gate_probability = 1.0;
  EXPECT_FALSE(is_valid(cfg));
  bool threw = false;
  try {
    Engine e(table(), cfg);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  EXPECT_TRUE(threw);
  cfg = {};
  cfg.lifecycle.coast_after_s = 20.0;  // after drop_after_s
  EXPECT_FALSE(is_valid(cfg));
}

}  // namespace sifted
