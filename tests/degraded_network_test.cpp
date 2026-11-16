// Degraded-network tests (M3): the multi-target scenarios sent through the
// impairment model (ADR 0008) and fed to net::Pipeline the way the daemon
// consumes them (tests/scenario_support.hpp: run_pipeline), scored against
// truth. Reports, per impairment level: per-target RMSE, missed
// target-instants, ID switches, false tracks and measurements dropped as out
// of order, with the reorder stage off (M2 policy) and on (ADR 0009).
//
// Recovery after an outage is defined in scenario_support.hpp
// (recovery_after) and docs/design.md section 9.
//
// The full sweep behind ADR 0009 (four reorder windows, three seeds per
// level) is DISABLED_ by default because it takes a while; run it with
//   build/release/tests/sifted_tests --gtest_also_run_disabled_tests
//       --gtest_filter='DegradedNetwork.DISABLED_Sweep'
// Bounds below are the measured baselines (printed by each test, recorded in
// docs/design.md section 9) plus margin. Engine parameters were not changed
// for these tests.

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "sifted/net/impairment.hpp"

namespace sifted {
namespace {

using net::ImpairmentConfig;
using net::LinkProfile;
using testing::PipelineRun;
using testing::Scenario;

constexpr std::int64_t kMs = 1'000'000;
constexpr std::int64_t kS = 1'000'000'000;

struct Level {
  const char* name;
  LinkProfile profile;
};

// Named impairment levels. Delays are one-way; the scenarios' sensors report
// at 1 Hz (radar scans) and 2 Hz (camera), so jitter of a few hundred ms
// reorders scans and splits them.
std::vector<Level> levels() {
  LinkProfile clean;
  LinkProfile mild;
  mild.loss = 0.01;
  mild.delay_ns = 20 * kMs;
  mild.jitter_ns = 50 * kMs;
  mild.duplicate = 0.01;
  LinkProfile moderate;
  moderate.loss = 0.03;
  moderate.p_good_to_bad = 0.01;
  moderate.p_bad_to_good = 0.25;
  moderate.loss_bad = 0.6;
  moderate.delay_ns = 50 * kMs;
  moderate.jitter_ns = 400 * kMs;
  moderate.duplicate = 0.02;
  LinkProfile severe;
  severe.loss = 0.08;
  severe.p_good_to_bad = 0.03;
  severe.p_bad_to_good = 0.2;
  severe.loss_bad = 0.8;
  severe.delay_ns = 100 * kMs;
  severe.jitter_ns = 800 * kMs;
  severe.duplicate = 0.05;
  return {{"clean", clean}, {"mild", mild}, {"moderate", moderate}, {"severe", severe}};
}

// formation_split is left out on purpose: its identity outcome is decided by
// a genuinely ambiguous assignment (see MultiTargetRegression.FormationSplit),
// so any perturbation of the input flips it with probability ~0.4 whatever
// the network does; it says nothing about the network. The sweep reports it.
const std::vector<std::string>& scenario_names() {
  static const std::vector<std::string> n = {"crossing_pair", "close_pass", "enter_leave"};
  return n;
}

// The reorder window the daemon uses by default (ADR 0009).
constexpr double kDefaultWindowS = 0.5;

PipelineRun run(const Scenario& sc, const LinkProfile& p, std::uint64_t seed, double window_s,
                std::vector<net::Outage> outages = {}) {
  ImpairmentConfig ic;
  ic.seed = seed;
  ic.global = p;
  ic.outages = std::move(outages);
  const auto arrivals = net::impair(testing::sent_frames(sc.measurements), ic);
  net::PipelineConfig pc;
  pc.reorder_window_s = window_s;
  return testing::run_pipeline(sc, arrivals, pc);
}

void print(const std::string& scenario, const char* level, double window, std::uint64_t seed,
           const PipelineRun& r) {
  const testing::Score& s = r.score;
  std::printf(
      "[ degraded ] %-15s %-8s window %.2f seed %llu: max RMSE %6.2f mean %6.2f missed %3zu/%zu "
      "(%.3f) idsw %zu false %zu ooo %llu/%llu (%.3f) late %llu ticks %llu\n",
      scenario.c_str(), level, window, static_cast<unsigned long long>(seed), s.max_rmse,
      s.mean_rmse, s.missed, s.eligible, s.missed_ratio(), s.id_switches, s.false_tracks,
      static_cast<unsigned long long>(s.out_of_order),
      static_cast<unsigned long long>(s.measurements),
      s.measurements ? static_cast<double>(s.out_of_order) / static_cast<double>(s.measurements)
                     : 0.0,
      static_cast<unsigned long long>(r.reorder_late), static_cast<unsigned long long>(r.ticks));
}

TEST(DegradedNetwork, CleanPipelineAddsOnlyLatency) {
  // With no impairment, the pipeline path makes the same identity decisions
  // as the direct engine path of the M2 regression; its RMSE is somewhat
  // higher only because the operator's picture lags: at truth time T the
  // radar scan stamped T is still being assembled (it is flushed by the next
  // frame or a 100 ms quiet tick), plus any reorder hold. This is the
  // reference the impaired levels are compared against.
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const testing::Score direct = testing::run_engine(sc);
    for (const double w : {0.0, kDefaultWindowS}) {
      const PipelineRun r = run(sc, {}, 1, w);
      print(name, "clean", w, 1, r);
      EXPECT_EQ(r.score.out_of_order, 0U) << name;
      EXPECT_EQ(r.score.id_switches, direct.id_switches) << name;
      EXPECT_EQ(r.score.false_tracks, direct.false_tracks) << name;
      EXPECT_EQ(r.score.missed, direct.missed) << name;
      EXPECT_GE(r.score.mean_rmse, direct.mean_rmse - 0.5) << name;
      EXPECT_LT(r.score.mean_rmse, 1.5 * direct.mean_rmse) << name << " window " << w;
    }
  }
}

struct Bound {
  double max_rmse_m;
  double max_missed_ratio;
  std::size_t max_id_switches;
  std::size_t max_false_tracks;
  double max_ooo_ratio;
};

void check_level(const char* level, double window, const Bound& b) {
  LinkProfile p;
  for (const Level& l : levels()) {
    if (std::string(l.name) == level) {
      p = l.profile;
    }
  }
  for (const auto& name : scenario_names()) {
    const Scenario sc = testing::load_scenario(name);
    const PipelineRun r = run(sc, p, 7, window);
    print(name, level, window, 7, r);
    const testing::Score& s = r.score;
    EXPECT_LT(s.max_rmse, b.max_rmse_m) << name;
    EXPECT_LE(s.missed_ratio(), b.max_missed_ratio) << name;
    EXPECT_LE(s.id_switches, b.max_id_switches) << name;
    EXPECT_LE(s.false_tracks, b.max_false_tracks) << name;
    EXPECT_LE(static_cast<double>(s.out_of_order),
              b.max_ooo_ratio * static_cast<double>(s.measurements))
        << name;
    EXPECT_EQ(s.rmse.size(), name == "enter_leave"  ? 4U
                             : name == "close_pass" ? 3U
                                                    : 2U)
        << "every target must still be tracked: " << name;
  }
}

// Measured (GCC 13 release, seed 7; worst scenario per column; the first
// line of each level is the M2 policy, the second the 0.5 s reorder window):
//
//   level     window  max RMSE [m]  missed  IDsw  false  out-of-order
//   mild      0       18.56         0.000   0     0      0.000
//   mild      0.5     19.07         0.000   0     0      0.000
//   moderate  0       20.10         0.027*  2*    0      0.112
//   moderate  0.5     19.56         0.003   0     0      0.000
//   severe    0       21.67         0.041   1     0      0.324
//   severe    0.5     22.29         0.013   0     0      0.002
//   (* formation_split, not part of the bounded set)
//
// The worst RMSE is always close_pass's manoeuvring target. Bounds: RMSE
// ~+35% (the M2 margin), missed ratio a few times the measured value, out of
// order a few times the measured value; identity errors are not allowed at
// mild and moderate, one at severe.
TEST(DegradedNetwork, MildWithoutReordering) { check_level("mild", 0.0, {25.0, 0.02, 0, 0, 0.01}); }
TEST(DegradedNetwork, MildWithReordering) {
  check_level("mild", kDefaultWindowS, {26.0, 0.02, 0, 0, 0.005});
}
TEST(DegradedNetwork, ModerateWithReordering) {
  check_level("moderate", kDefaultWindowS, {26.0, 0.03, 0, 0, 0.005});
}
TEST(DegradedNetwork, SevereWithReordering) {
  check_level("severe", kDefaultWindowS, {30.0, 0.05, 1, 1, 0.01});
}

TEST(DegradedNetwork, ReorderingRecoversDataThatDroppingLoses) {
  // The reason for ADR 0009, as a test: at moderate and severe jitter the M2
  // policy loses a large share of measurements as OutOfOrder; the reorder
  // stage loses almost none, and tracks better.
  for (const char* level : {"moderate", "severe"}) {
    LinkProfile p;
    for (const Level& l : levels()) {
      if (std::string(l.name) == level) {
        p = l.profile;
      }
    }
    for (const auto& name : scenario_names()) {
      const Scenario sc = testing::load_scenario(name);
      const PipelineRun drop = run(sc, p, 7, 0.0);
      const PipelineRun hold = run(sc, p, 7, kDefaultWindowS);
      print(name, level, 0.0, 7, drop);
      print(name, level, kDefaultWindowS, 7, hold);
      EXPECT_GT(drop.score.out_of_order, 10 * (hold.score.out_of_order + 1))
          << name << " " << level;
      EXPECT_LE(hold.score.missed, drop.score.missed) << name << " " << level;
    }
  }
}

TEST(DegradedNetwork, RecoveryAfterOutage) {
  // A link outage of every sensor, mid-run, while all targets are covered,
  // on top of the mild level. Expected (general, not per scenario), with
  // `gap` = outage end - the track's last update before the outage:
  //   * gap + 2 s (the first revisit after the link returns, plus one missed
  //     detection) within drop_after_s (10 s), and the target flies straight
  //     through the gap: its coasting track holds it (same id). One that
  //     manoeuvres during the gap may fall out of its gate and be re-initiated;
  //   * gap beyond drop_after_s: the track is dropped as Stale at the first
  //     measurement after the link returns, and the target re-initiated;
  //   * in between (e.g. the 8 s outage) either can happen; it is reported;
  //   * either way every target is tracked again within a few revisits.
  for (const std::int64_t length_s : {5, 8, 15}) {
    for (const std::string name : {"crossing_pair", "close_pass", "enter_leave"}) {
      const Scenario sc = testing::load_scenario(name);
      const std::int64_t t0 = sc.truth.front().time_ns;
      const std::int64_t start = t0 + 30 * kS;
      const std::int64_t end = start + length_s * kS;
      for (const double w : {0.0, kDefaultWindowS}) {
        const PipelineRun r = run(sc, levels()[1].profile, 11, w, {{start, end}});
        const testing::Recovery rec = testing::recovery_after(r.score, start, end);
        std::printf(
            "[ recovery ] %-13s outage %2llds window %.2f: targets %zu, recovered after %.1f s, "
            "survived %zu, re-initiated %zu\n",
            name.c_str(), static_cast<long long>(length_s), w, rec.targets, rec.recovery_s,
            rec.survived, rec.reinitiated);
        ASSERT_GT(rec.targets, 0U);
        EXPECT_GE(rec.recovery_s, 0.0) << "never recovered";
        // Measured: 1-3 s through a coast, 3-4 s after a re-initiation
        // (confirm_hits = 3 at 1 Hz), plus the reorder hold.
        EXPECT_LE(rec.recovery_s, 5.0 + w) << name;
        const double drop_after = EngineConfig{}.lifecycle.drop_after_s;
        for (const auto& [tgt, survived] : rec.target_survived) {
          const double gap = static_cast<double>(end - rec.last_update_before.at(tgt)) * 1e-9;
          if (gap > drop_after) {
            EXPECT_FALSE(survived) << name << " target " << tgt;
          } else if (gap + 2.0 <= drop_after &&
                     testing::velocity_change(sc, tgt, start, end) < 1.0) {
            EXPECT_TRUE(survived) << name << " target " << tgt << " flew straight";
          }
        }
      }
    }
  }
}

TEST(DegradedNetwork, DemoScenarioAsInTheRunbook) {
  // docs/demo.md: the demo scenario through the moderate level with an 8 s
  // outage at t = 70 s and the daemon's 0.5 s reorder window. Every target
  // is tracked, the outage is recovered from, and nothing false is shown.
  const Scenario sc = testing::load_scenario("demo");
  const std::int64_t start = sc.truth.front().time_ns + 70 * kS;
  const std::int64_t end = start + 8 * kS;
  const PipelineRun r = run(sc, levels()[2].profile, 7, kDefaultWindowS, {{start, end}});
  print("demo", "moderate", kDefaultWindowS, 7, r);
  const testing::Recovery rec = testing::recovery_after(r.score, start, end);
  std::printf("[ recovery ] demo outage 8s: targets %zu, recovered after %.1f s, survived %zu\n",
              rec.targets, rec.recovery_s, rec.survived);
  EXPECT_EQ(r.score.rmse.size(), 5U);
  EXPECT_EQ(r.score.false_tracks, 0U);
  EXPECT_LE(r.score.missed_ratio(), 0.05);
  EXPECT_GE(rec.recovery_s, 0.0);
  EXPECT_LE(rec.recovery_s, 5.5);
}

TEST(DegradedNetwork, DISABLED_Sweep) {
  for (const Level& l : levels()) {
    for (const double w : {0.0, 0.25, 0.5, 1.0, 2.0}) {
      for (const std::string name :
           {"crossing_pair", "close_pass", "enter_leave", "formation_split"}) {
        const Scenario sc = testing::load_scenario(name);
        for (const std::uint64_t seed : {7ULL, 8ULL, 9ULL}) {
          print(name, l.name, w, seed, run(sc, l.profile, seed, w));
        }
      }
    }
  }
}

}  // namespace
}  // namespace sifted
