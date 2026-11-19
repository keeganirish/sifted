// Real-data evaluation A scoring (eval/sifted/eval/realdata.hpp) on small,
// hand-checkable synthetic runs: an aircraft flying due east along the
// equator at 100 m/s, reporting once a second without noise.

#include "sifted/eval/realdata.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace sifted {
namespace {

using eval::Estimator;
using eval::HoldoutRun;

constexpr double kMPerDegLon = 111319.49079327357;  // WGS-84 equator
constexpr SensorId kSensor = 100;

EngineConfig adsb_config() {  // app/launcher.py ADSB_TRACKER_ARGS
  EngineConfig c;
  c.initial_velocity_sigma = 250.0;
  c.lifecycle.tentative_timeout_s = 90.0;
  c.lifecycle.coast_after_s = 60.0;
  c.lifecycle.drop_after_s = 180.0;
  c.lifecycle.max_position_sigma_m = 10000.0;
  return c;
}

Measurement report(std::uint32_t seq, double t_s, double lon) {
  Measurement m;
  m.sensor_id = kSensor;
  m.kind = SensorKind::GeoPosition;
  m.seq = seq;
  m.time = Timestamp{static_cast<std::int64_t>(std::llround(t_s * 1e9))};
  m.z = {0.0, lon, 0.0};  // on the ellipsoid, so east distance is 100 m/s exactly
  m.sigma = {50.0 / 111320.0, 50.0 / 111320.0, 50.0};
  return m;
}

// Reports at t = 0..n-1 s; those with withheld(i) go to the withheld list.
template <class Withheld, class Label>
HoldoutRun eastbound(int n, Withheld withheld, Label label) {
  HoldoutRun run;
  run.sensors =
      parse_sensor_table("sifted-sensors 1\nsensor 100 geo_position 0.0 0.0 0.0\n").value();
  run.origin = {0.0, 0.0, 0.0};
  std::uint32_t seq = 0;
  for (int i = 0; i < n; ++i) {
    const double lon = 100.0 * i / kMPerDegLon;
    if (withheld(i)) {
      run.withheld.push_back(
          {static_cast<std::int64_t>(i) * 1'000'000'000, label(i), {0.0, lon, 0.0}});
    } else {
      run.measurements.push_back(report(seq++, i, lon));
      run.key.push_back({label(i), 100.0, 90.0});
    }
  }
  return run;
}

TEST(RealData, QuantilesAreNearestRank) {
  std::vector<double> v;
  for (int i = 100; i >= 1; --i) {
    v.push_back(i);
  }
  const eval::Quantiles q = eval::quantiles(v);
  EXPECT_EQ(q.n, 100U);
  EXPECT_EQ(q.median, 50.0);
  EXPECT_EQ(q.p95, 95.0);
  EXPECT_EQ(q.p99, 99.0);
  EXPECT_DOUBLE_EQ(q.mean, 50.5);
  EXPECT_EQ(eval::quantiles({7.0}).p99, 7.0);
  EXPECT_EQ(eval::quantiles({}).n, 0U);
}

TEST(RealData, PredictionAgainstBaselinesOnStraightFlight) {
  const HoldoutRun run =
      eastbound(200, [](int i) { return i >= 100 && i < 110; }, [](int) { return 1; });
  const eval::HoldoutScore s = eval::score_holdout(run, adsb_config());
  EXPECT_EQ(s.withheld, 10U);
  EXPECT_EQ(s.no_prior_report, 0U);
  EXPECT_EQ(s.no_track, 0U);
  const auto& hold = s.errors[static_cast<std::size_t>(Estimator::Hold)];
  const auto& dr = s.errors[static_cast<std::size_t>(Estimator::DeadReckoning)];
  const auto& tracker = s.errors[static_cast<std::size_t>(Estimator::Tracker)];
  ASSERT_EQ(hold.size(), 10U);
  ASSERT_EQ(dr.size(), 10U);
  ASSERT_EQ(tracker.size(), 10U);
  for (std::size_t k = 0; k < 10; ++k) {
    const double h = static_cast<double>(k + 1);  // last fed report at t = 99 s
    EXPECT_DOUBLE_EQ(hold[k].horizon_s, h);
    EXPECT_NEAR(hold[k].error_m, 100.0 * h, 0.05);  // the aircraft moved on 100 m/s
    EXPECT_LT(dr[k].error_m, 0.05);                 // exact velocity, straight path
    EXPECT_LT(tracker[k].error_m, 5.0);             // a converged CV filter on a line
  }
  // One aircraft, one confirmed track, every report after the first three on it.
  EXPECT_EQ(s.tracks_started, 1U);
  EXPECT_EQ(s.confirmed_tracks, 1U);
  EXPECT_EQ(s.fragments, 0U);
  EXPECT_EQ(s.never_confirmed, 0U);
  EXPECT_EQ(s.swaps, 0U);
  EXPECT_EQ(s.coverage_eligible(), 190U - 3U);
  EXPECT_EQ(s.covered(), 190U - 3U);
  EXPECT_EQ(s.nis.n, 189U - eval::kNisBurnInUpdates);  // 190 fed: 1 initiates, 189 associate
  EXPECT_LT(s.nis.average(), 0.5);                     // noise-free reports against a 50 m sigma
  EXPECT_EQ(s.outcomes.at(MeasurementOutcome::Updated), 189U);
}

TEST(RealData, WithheldBeforeFirstReportHasNoPrior) {
  const HoldoutRun run = eastbound(50, [](int i) { return i < 5; }, [](int) { return 1; });
  const eval::HoldoutScore s = eval::score_holdout(run, adsb_config());
  EXPECT_EQ(s.withheld, 5U);
  EXPECT_EQ(s.no_prior_report, 5U);
  EXPECT_TRUE(s.errors[0].empty());
}

TEST(RealData, ContinuityCountsSwapsAndMixedTracks) {
  // Two labels on one kinematic path: whatever the tracker does, it cannot
  // tell them apart, and the scorer must see one track shared by both.
  const HoldoutRun run = eastbound(40, [](int) { return false; }, [](int i) { return 1 + i % 2; });
  const eval::HoldoutScore s = eval::score_holdout(run, adsb_config());
  EXPECT_EQ(s.confirmed_tracks, 1U);
  EXPECT_EQ(s.mixed_tracks, 1U);
  EXPECT_EQ(s.swaps, 39U);
  EXPECT_EQ(s.inconsistent_tracks, 1U);  // 20 of 40 each: no majority
  EXPECT_EQ(s.aircraft.size(), 2U);
  EXPECT_EQ(s.fragments, 0U);
}

TEST(RealData, TrackBreakIsAFragment) {
  // A 400 s silence exceeds drop_after_s (180 s): the aircraft gets a second track.
  HoldoutRun run = eastbound(40, [](int) { return false; }, [](int) { return 1; });
  const HoldoutRun later = eastbound(40, [](int) { return false; }, [](int) { return 1; });
  for (std::size_t i = 0; i < later.measurements.size(); ++i) {
    Measurement m = later.measurements[i];
    m.time.ns += 1000'000'000'000;
    m.seq += 40;
    run.measurements.push_back(m);
    run.key.push_back(later.key[i]);
  }
  const eval::HoldoutScore s = eval::score_holdout(run, adsb_config());
  EXPECT_EQ(s.tracks_started, 2U);
  EXPECT_EQ(s.confirmed_tracks, 2U);
  EXPECT_EQ(s.fragments, 1U);
}

TEST(RealData, LoaderIsStrict) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::path(::testing::TempDir()) / "sifted_holdout_run";
  fs::remove_all(dir);
  fs::create_directories(dir);
  auto write = [&](const char* name, const std::string& text) {
    std::ofstream(dir / name, std::ios::binary) << text;
  };
  write("sensors.cfg", "sifted-sensors 1\nsensor 100 geo_position 0.0 0.0 0.0\n");
  write("origin.txt", "0.0 0.0 0.0\n");
  write("frames.hex", "");
  write("feed_key.csv", "frame,aircraft,speed_mps,track_deg\n");
  write("withheld.csv", "time_ns,aircraft,lat,lon,alt_m\n5,1,0.0,0.001,0.0\n");
  ASSERT_TRUE(eval::is_holdout_dir(dir.string()));
  const HoldoutRun ok = eval::load_holdout_dir(dir.string());
  EXPECT_EQ(ok.withheld.size(), 1U);
  EXPECT_EQ(eval::score_holdout(ok, adsb_config()).no_prior_report, 1U);

  write("origin.txt", "91.0 0.0 0.0\n");
  EXPECT_THROW(eval::load_holdout_dir(dir.string()), std::runtime_error);
  write("origin.txt", "0.0 0.0 0.0\n");
  write("withheld.csv", "time_ns,aircraft,lat,lon\n");
  EXPECT_THROW(eval::load_holdout_dir(dir.string()), std::runtime_error);
  write("withheld.csv", "time_ns,aircraft,lat,lon,alt_m\n9,1,0,0,0\n5,1,0,0,0\n");
  EXPECT_THROW(eval::load_holdout_dir(dir.string()), std::runtime_error);  // out of order
  write("withheld.csv", "time_ns,aircraft,lat,lon,alt_m\n");
  write("feed_key.csv", "frame,aircraft,speed_mps,track_deg\n0,1,,\n");
  EXPECT_THROW(eval::load_holdout_dir(dir.string()), std::runtime_error);  // no frame 0
  write("feed_key.csv", "frame,aircraft,speed_mps,track_deg\n");
  write("frames.hex", "zz\n");
  EXPECT_THROW(eval::load_holdout_dir(dir.string()), std::runtime_error);
  fs::remove_all(dir);
}

}  // namespace
}  // namespace sifted
