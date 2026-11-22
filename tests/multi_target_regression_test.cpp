// Multi-target, multi-sensor ground-truth regression: simulator output ->
// C++ decoder -> Engine (scan-grouped, as the ingest path delivers it),
// scored against the simulator's truth.
//
// Fixtures: tests/fixtures/scenarios/{crossing_pair,close_pass,enter_leave,
// formation_split}/, regenerated with
//   cd sim && python3 -m sifted_sim.generate <name> ../tests/fixtures/scenarios/<name>
// The engine sees only the sensor table and the frames. Truth lists, once per
// second, every target that exists, flagged when it is inside radar coverage
// (truth.csv layout 2, with a target_id and a covered flag; see
// tests/fixtures/README.md). Loading and the CLEAR-MOT style scoring live in
// eval/sifted/eval/ (reached through tests/scenario_support.hpp, and shared
// with the degraded-network and fault tests and sifted_eval); the definitions
// are documented there.
//
// Thresholds: the raw radar error is 15-30 m across these geometries and the
// M1 filter averages it to ~0.5-0.7x on a single target; fusing the camera
// lowers it further. The bounds below are the measured baselines plus margin
// (recorded with the tests at the end of this file), so seed-level noise does
// not flake while a broken component fails; see the sensitivity checks noted
// there.

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "sifted/core/engine.hpp"

namespace sifted {
namespace {

using testing::load_scenario;
using testing::run_engine;
using testing::Scenario;
using testing::Score;

Score run(const Scenario& sc, const EngineConfig& cfg = {}) { return run_engine(sc, cfg); }

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

void check_model(const std::string& scenario, const Bounds& b, std::size_t expected_targets,
                 estimation::MotionModel model) {
  const std::string name = scenario + " (" + std::string(estimation::to_string(model)) + ")";
  EngineConfig cfg;
  cfg.motion_model = model;
  const Scenario sc = load_scenario(scenario);
  EXPECT_FALSE(sc.measurements.empty());
  const Score s = run(sc, cfg);
  const Score radar_only = run(without_bearings(sc), cfg);
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
  const Score again = run(sc, cfg);
  EXPECT_EQ(again.trace.size(), s.trace.size());
  for (std::size_t i = 0; i < s.trace.size() && i < again.trace.size(); ++i) {
    ASSERT_EQ(again.trace[i].id, s.trace[i].id);
    ASSERT_TRUE(again.trace[i].state == s.trace[i].state) << "estimate " << i;
    ASSERT_TRUE(again.trace[i].covariance == s.trace[i].covariance) << "estimate " << i;
  }
}

// The same bounds for both estimators: the IMM (the default, ADR 0014) and the
// legacy constant-velocity filter the bounds were set with.
void check(const std::string& name, const Bounds& b, std::size_t expected_targets) {
  for (const auto m : {estimation::MotionModel::Imm, estimation::MotionModel::ConstantVelocity}) {
    check_model(name, b, expected_targets, m);
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
// With the IMM (the default since ADR 0014; same bounds, both estimators run):
//
//   crossing_pair  7.09 7.09                    0/172   0     0      338/338    7.09/9.16  (0.77)
//   close_pass     8.60 5.94 10.27              0/258   0     0      425/425    8.27/11.44 (0.72)
//   enter_leave    (per target: evaluation.md)  0/302   0     0      578/666    7.30/9.76  (0.75)
//   formation_split (per target: evaluation.md) 0/258   0     0      507/507   10.41/26.45 (0.39)
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
//   greedy instead of GNN    no change on these three: they never present an
//                            ambiguity greedy gets wrong. formation_split
//                            (below) was added in M3 for exactly that.
TEST(MultiTargetRegression, CrossingPair) { check("crossing_pair", {12.0, 0.02, 0, 0, 0.995}, 2); }

TEST(MultiTargetRegression, ClosePass) { check("close_pass", {16.0, 0.02, 0, 0, 0.99}, 3); }

TEST(MultiTargetRegression, EnterLeave) { check("enter_leave", {14.0, 0.05, 0, 0, 0.8}, 4); }

// formation_split (M3): three targets 45 m apart (about 3 radar range sigmas)
// in line abreast for 40 s, then the outer two turn away. Every scan offers
// each track two or three gated detections, so the assignment decides
// identity on every scan. Measured (GCC 13, real Eigen):
//
//   method   per-target RMSE [m]          missed  IDsw  false  bearings  fused/radar-only
//   GNN      11.54 11.58 10.42             0/258   0     0      507/507   11.18/27.78 (0.40)
//   greedy   max 35.49, mean 27.27         1/258   2     0
//
// Greedy takes the single closest pair first and so now and then hands a
// track its neighbour's detection; the swap stays invisible in formation and
// surfaces at the split as a track following the wrong target (ID switches,
// RMSE of about one lane spacing). How general that is, rather than a
// property of this fixture's noise: regenerating the scenario with four
// other noise seeds, greedy broke identity in all of them (2-3 ID switches,
// RMSE 36-41 m) and GNN in two (the formation is genuinely ambiguous at 45 m,
// so an optimal assignment is not infallible either); with three such
// formations per run, totals over five seeds were 28 ID switches for greedy
// against 12 for GNN, and greedy's mean RMSE was higher in four of the five
// runs and equal in the fifth. The committed fixture is the scenario's
// default seed; the bounds are its GNN baseline plus margin, and the test
// checks that greedy breaks them. (Radar-only also loses identity here --
// the camera's bearings from the side are what separate the lanes -- which
// the fused/radar-only ratio check records.)
TEST(MultiTargetRegression, FormationSplit) {
  check("formation_split", {16.0, 0.02, 0, 0, 0.9}, 3);

  // The GNN-versus-greedy comparison was built with, and is kept on, the
  // constant-velocity filter: with the IMM this fixture's noise draw happens
  // to stay unambiguous for greedy too (ADR 0014; over ten other seeds the
  // IMM breaks formation identity more often, not less).
  EngineConfig greedy;
  greedy.motion_model = estimation::MotionModel::ConstantVelocity;
  greedy.association.method = association::AssignmentMethod::Greedy;
  const Score g = run(load_scenario("formation_split"), greedy);
  std::printf(
      "[ scenario ] formation_split with greedy assignment: id switches %zu, max RMSE "
      "%.2f m, mean %.2f m\n",
      g.id_switches, g.max_rmse, g.mean_rmse);
  EXPECT_TRUE(g.id_switches > 0 || g.max_rmse >= 16.0)
      << "greedy assignment should fail this scenario; if it no longer does, the scenario "
         "stopped testing what GNN buys";
}

}  // namespace sifted
