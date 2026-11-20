// The gate screen (estimation::may_gate, M3 performance work) must never
// change a result: every scenario, with and without clutter floods and
// faults, gives bit-identical outcomes, estimates and decision traces with
// the screen on and off. The engine fuzz target checks the same property on
// arbitrary input.

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "sifted/estimation/measurement_model.hpp"
#include "sifted/estimation/range_bearing.hpp"
#include "sifted/estimation/track_filter.hpp"
#include "sifted/eval/faults.hpp"

namespace sifted {
namespace {

void expect_identical(const SensorTable& sensors, const std::vector<Measurement>& ms,
                      EngineConfig cfg, const std::string& what) {
  cfg.association.gate_screen = true;
  Engine on(sensors, cfg);
  cfg.association.gate_screen = false;
  Engine off(sensors, cfg);
  std::vector<MeasurementResult> r_on;
  std::vector<MeasurementResult> r_off;
  on.on_scan(ms, r_on);
  off.on_scan(ms, r_off);
  ASSERT_EQ(r_on, r_off) << what;
  const auto t_on = on.tracks();
  const auto t_off = off.tracks();
  ASSERT_EQ(t_on.size(), t_off.size()) << what;
  for (std::size_t i = 0; i < t_on.size(); ++i) {
    ASSERT_EQ(t_on[i].id, t_off[i].id) << what;
    ASSERT_TRUE(t_on[i].state == t_off[i].state) << what;
    ASSERT_TRUE(t_on[i].covariance == t_off[i].covariance) << what;
  }
  for (TrackId id = 1; id <= on.tracks_started(); ++id) {
    ASSERT_EQ(on.explain(id), off.explain(id)) << what << " track " << id;  // full trace
  }
}

// Both estimators: for the IMM the screen tests every mode (ADR 0014).
TEST(GateScreen, NeverChangesResults) {
  for (const auto model :
       {estimation::MotionModel::ConstantVelocity, estimation::MotionModel::Imm}) {
    EngineConfig base;
    base.motion_model = model;
    const std::string m = " (" + std::string(estimation::to_string(model)) + ")";
    for (const std::string name : {"crossing_pair", "close_pass", "enter_leave", "formation_split",
                                   "turn", "straight_crossing"}) {
      const testing::Scenario sc = testing::load_scenario(name);
      expect_identical(sc.sensors, sc.measurements, base, name + m);
      if (name == "turn" || name == "straight_crossing") {
        continue;  // single-sensor fixtures: no clutter injection below
      }
      SensorId radar = 0;
      for (const auto& s : sc.sensors.sensors()) {
        if (s.kind == SensorKind::RangeBearing) {
          radar = s.id;
        }
      }
      EngineConfig crowded = base;
      crowded.max_tracks = 4096;
      expect_identical(sc.sensors, testing::faults::clutter(sc.measurements, radar, 20, 8000.0, 3),
                       crowded, name + " + clutter" + m);
      expect_identical(sc.sensors, testing::faults::bias(sc.measurements, radar, 50.0), base,
                       name + " + bias" + m);
    }
  }
}

// The IMM's statistic is never below the smallest per-mode d^2, so "no mode
// may gate" proves "the pair cannot gate". Random IMM states with genuinely
// different modes (built by updating with random measurements), random
// range-bearing and position measurements.
TEST(GateScreen, ImmBoundIsConservativeOnRandomPairs) {
  net::SplitMix64 rng(777);
  estimation::ImmParams params;
  std::size_t skipped = 0;
  std::size_t checked = 0;
  for (int i = 0; i < 20000; ++i) {
    estimation::StateVector x;
    x << 20000.0 * rng.uniform() - 10000.0, 20000.0 * rng.uniform() - 10000.0,
        200.0 * rng.uniform() - 100.0, 200.0 * rng.uniform() - 100.0;
    estimation::StateCovariance P = estimation::StateCovariance::Identity();
    P.diagonal() << 2500.0 * rng.uniform() + 1.0, 2500.0 * rng.uniform() + 1.0,
        400.0 * rng.uniform() + 1.0, 400.0 * rng.uniform() + 1.0;
    auto created = estimation::ImmFilter::create(x, P, params);
    ASSERT_TRUE(created);
    estimation::TrackFilter f(created.value());
    // A few predict / update cycles so the modes separate.
    for (int k = 0; k < 3; ++k) {
      ASSERT_EQ(f.predict(0.5 + 2.0 * rng.uniform()), estimation::FilterStatus::Ok);
      Measurement u;
      u.sensor_id = 1;
      u.kind = SensorKind::RangeBearing;
      const estimation::RangeBearing z =
          estimation::predict_range_bearing(f.state(), SensorPose{0.0, 0.0, 0.0});
      u.z = {z.range_m + 200.0 * (rng.uniform() - 0.5),
             z.bearing_rad + 0.02 * (rng.uniform() - 0.5), 0.0};
      u.sigma = {10.0, 0.0035, 0.0};
      const auto uc = estimation::make_context(u, SensorPose{0.0, 0.0, 0.0}, std::nullopt);
      ASSERT_TRUE(uc);
      if (const auto g = f.gate(*uc)) {
        (void)f.update(*uc, g->lin);
      }
    }
    Measurement m;
    m.sensor_id = 1;
    m.kind = SensorKind::RangeBearing;
    m.z = {20000.0 * rng.uniform(), 6.28 * rng.uniform() - 3.14, 0.0};
    m.sigma = {1.0 + 50.0 * rng.uniform(), 1e-4 + 0.02 * rng.uniform(), 0.0};
    const SensorPose pose{1000.0 * rng.uniform(), -1000.0 * rng.uniform(), rng.uniform() - 0.5};
    const auto ctx = estimation::make_context(m, pose, std::nullopt);
    ASSERT_TRUE(ctx);
    const double limit = 4.0 * 13.8155;
    const auto gs = estimation::make_screen(*ctx);
    bool may = false;
    double min_mode = std::numeric_limits<double>::infinity();
    for (std::size_t c = 0; c < f.components(); ++c) {
      const estimation::StateCovariance& Pc = f.component_covariance(c);
      const double a = Pc(0, 0);
      const double b = Pc(0, 1);
      const double d = Pc(1, 1);
      const double lambda = 0.5 * (a + d) + std::sqrt(0.25 * (a - d) * (a - d) + b * b);
      may = may || estimation::may_gate(gs, f.component_state(c).head<2>(), lambda, limit);
      if (const auto lin = estimation::linearize(*ctx, f.component_state(c))) {
        auto single = estimation::ConstantVelocityFilter::create(f.component_state(c), Pc, 0.0);
        if (single) {
          if (const auto d2 = estimation::mahalanobis2(single.value(), *lin)) {
            min_mode = std::min(min_mode, *d2);
          }
        }
      }
    }
    const auto g = f.gate(*ctx);
    if (g) {
      ++checked;
      EXPECT_GE(g->d2, min_mode * (1.0 - 1e-12)) << "mixture statistic below its lower bound";
    }
    if (!may) {
      ++skipped;
      ASSERT_TRUE(!g || g->d2 > limit) << "screened out a pair with d2 " << g->d2;
    }
  }
  EXPECT_GT(skipped, 5000U);
  EXPECT_GT(checked, 15000U);
}

TEST(GateScreen, BoundIsConservativeOnRandomPairs) {
  // Direct check of the inequality on many random geometries: whenever the
  // screen says "skip", the full computation is above the limit.
  net::SplitMix64 rng(12345);
  std::size_t skipped = 0;
  for (int i = 0; i < 200000; ++i) {
    Measurement m;
    m.sensor_id = 1;
    m.kind = SensorKind::RangeBearing;
    m.time = Timestamp{0};
    m.z = {20000.0 * rng.uniform(), 6.28 * rng.uniform() - 3.14, 0.0};
    m.sigma = {1.0 + 50.0 * rng.uniform(), 1e-4 + 0.02 * rng.uniform(), 0.0};
    const SensorPose pose{1000.0 * rng.uniform(), -1000.0 * rng.uniform(), rng.uniform() - 0.5};
    const auto ctx = estimation::make_context(m, pose, std::nullopt);
    ASSERT_TRUE(ctx);
    estimation::StateVector x;
    x << 30000.0 * rng.uniform() - 15000.0, 30000.0 * rng.uniform() - 15000.0, 0.0, 0.0;
    const double sx = 1.0 + 500.0 * rng.uniform();
    const double sy = 1.0 + 500.0 * rng.uniform();
    const double rho = 1.8 * rng.uniform() - 0.9;
    estimation::StateCovariance P = estimation::StateCovariance::Identity() * 100.0;
    P(0, 0) = sx * sx;
    P(1, 1) = sy * sy;
    P(0, 1) = P(1, 0) = rho * sx * sy;
    auto f = estimation::ConstantVelocityFilter::create(x, P, 4.0);
    ASSERT_TRUE(f);
    const double a = P(0, 0);
    const double b = P(0, 1);
    const double d = P(1, 1);
    const double lambda = 0.5 * (a + d) + std::sqrt(0.25 * (a - d) * (a - d) + b * b);
    const double limit = 4.0 * 13.8155;
    const auto gs = estimation::make_screen(*ctx);
    if (estimation::may_gate(gs, x.head<2>(), lambda, limit)) {
      continue;
    }
    ++skipped;
    const auto lin = estimation::linearize(*ctx, x);
    if (!lin) {
      continue;
    }
    const auto d2 = estimation::mahalanobis2(f.value(), *lin);
    ASSERT_TRUE(!d2 || *d2 > limit) << "screened out a pair with d2 " << *d2;
  }
  EXPECT_GT(skipped, 100000U);  // and it does prune the vast majority
}

}  // namespace
}  // namespace sifted
