// GOSPA (eval/sifted/eval/gospa.hpp): hand-computed examples for each
// component, empty and rectangular sets, the cut-off, optimal (not greedy)
// assignment, and the metric axioms on small randomised sets.

#include "sifted/eval/gospa.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "sifted/net/impairment.hpp"

namespace sifted::eval {
namespace {

constexpr GospaParams kDefault{};  // c = 100 m, p = 2: c^p / 2 = 5000 m^2

TEST(Gospa, DefaultsAreTheDocumentedOnes) {
  EXPECT_EQ(kDefault.c, 100.0);
  EXPECT_EQ(kDefault.p, 2.0);
}

TEST(Gospa, BothEmptyIsZero) {
  const Gospa g = gospa({}, {});
  EXPECT_EQ(g.total, 0.0);
  EXPECT_EQ(g.assigned, 0U);
}

TEST(Gospa, LocalisationOnly) {
  const Gospa g = gospa({{0, 0}}, {{3, 4}});  // d = 5
  EXPECT_DOUBLE_EQ(g.localisation, 25.0);
  EXPECT_EQ(g.missed, 0.0);
  EXPECT_EQ(g.false_tracks, 0.0);
  EXPECT_DOUBLE_EQ(g.total, 5.0);
  EXPECT_EQ(g.assignment, std::vector<int>{0});
}

TEST(Gospa, MissedTargetCostsHalfCToThePOnEmptyEstimates) {
  const Gospa g = gospa({{0, 0}, {500, 500}}, {});
  EXPECT_EQ(g.missed_count, 2U);
  EXPECT_DOUBLE_EQ(g.missed, 10000.0);
  EXPECT_DOUBLE_EQ(g.total, 100.0);
}

TEST(Gospa, FalseTracksOnEmptyTruth) {
  const Gospa g = gospa({}, {{0, 0}});
  EXPECT_EQ(g.false_count, 1U);
  EXPECT_DOUBLE_EQ(g.false_tracks, 5000.0);
  EXPECT_DOUBLE_EQ(g.total, std::sqrt(5000.0));
}

TEST(Gospa, RectangularProblemsCountTheSurplus) {
  // Two targets, one estimate near the first: 36 + one miss.
  const Gospa a = gospa({{0, 0}, {1000, 0}}, {{0, 6}});
  EXPECT_DOUBLE_EQ(a.localisation, 36.0);
  EXPECT_EQ(a.missed_count, 1U);
  EXPECT_EQ(a.false_count, 0U);
  EXPECT_DOUBLE_EQ(a.total, std::sqrt(5036.0));
  // One target, two estimates: 64 + one false track.
  const Gospa b = gospa({{0, 0}}, {{1000, 0}, {8, 0}});
  EXPECT_DOUBLE_EQ(b.localisation, 64.0);
  EXPECT_EQ(b.false_count, 1U);
  EXPECT_EQ(b.assignment, std::vector<int>{1});
  EXPECT_DOUBLE_EQ(b.total, std::sqrt(5064.0));
}

TEST(Gospa, BeyondTheCutOffIsOneMissPlusOneFalse) {
  const Gospa far = gospa({{0, 0}}, {{150, 0}});
  EXPECT_EQ(far.assigned, 0U);
  EXPECT_DOUBLE_EQ(far.total, 100.0);  // sqrt(5000 + 5000) = c
  const Gospa near = gospa({{0, 0}}, {{99, 0}});
  EXPECT_EQ(near.assigned, 1U);
  EXPECT_DOUBLE_EQ(near.total, 99.0);
}

TEST(Gospa, AssignmentIsOptimalNotGreedy) {
  // Greedy would pair (10,0)-(6,0) first (16) and then pay 256 for the other.
  const Gospa g = gospa({{0, 0}, {10, 0}}, {{6, 0}, {16, 0}});
  EXPECT_DOUBLE_EQ(g.localisation, 72.0);
  EXPECT_EQ(g.assignment, (std::vector<int>{0, 1}));
}

TEST(Gospa, OrderOneAndOtherCutOff) {
  const GospaParams p1{.c = 10.0, .p = 1.0};
  const Gospa g = gospa({{0, 0}, {100, 0}}, {{3, 4}}, p1);  // 5 + 10/2
  EXPECT_DOUBLE_EQ(g.localisation, 5.0);
  EXPECT_DOUBLE_EQ(g.missed, 5.0);
  EXPECT_DOUBLE_EQ(g.total, 10.0);
}

TEST(Gospa, ComponentsAddUpToTheTotal) {
  const Gospa g = gospa({{0, 0}, {50, 50}, {900, 0}}, {{1, 2}, {52, 47}, {300, 300}, {700, 0}});
  EXPECT_NEAR(std::pow(g.total, 2.0), g.localisation + g.missed + g.false_tracks, 1e-9);
  EXPECT_EQ(g.assigned + g.missed_count, 3U);
  EXPECT_EQ(g.assigned + g.false_count, 4U);
}

TEST(Gospa, RejectsBadParametersAndCoordinates) {
  EXPECT_THROW(gospa({}, {}, {.c = 0.0, .p = 2.0}), std::invalid_argument);
  EXPECT_THROW(gospa({}, {}, {.c = 100.0, .p = 0.5}), std::invalid_argument);
  EXPECT_THROW(gospa({{std::numeric_limits<double>::quiet_NaN(), 0}}, {}), std::invalid_argument);
  EXPECT_THROW(gospa({}, {{0, std::numeric_limits<double>::infinity()}}), std::invalid_argument);
}

TEST(Gospa, SumAveragesOverInstants) {
  GospaSum s;
  s.add(gospa({{0, 0}}, {{3, 4}}), 2.0);  // 25
  s.add(gospa({{0, 0}}, {}), 2.0);        // 5000
  EXPECT_EQ(s.instants, 2U);
  EXPECT_DOUBLE_EQ(s.mean_total(2.0), std::sqrt(2512.5));
  EXPECT_EQ(s.missed_count, 1U);
  EXPECT_EQ(GospaSum{}.mean_total(2.0), 0.0);
}

std::vector<Point2> random_set(net::SplitMix64& rng) {
  std::vector<Point2> s(rng.below(5));
  for (Point2& p : s) {
    p = {300.0 * rng.uniform(), 300.0 * rng.uniform()};  // spans the 100 m cut-off
  }
  return s;
}

TEST(GospaProperties, IsAMetricOnRandomSets) {
  net::SplitMix64 rng(20260101);
  for (const GospaParams params : {GospaParams{}, GospaParams{.c = 40.0, .p = 1.0}}) {
    for (int trial = 0; trial < 500; ++trial) {
      const auto x = random_set(rng);
      const auto y = random_set(rng);
      const auto z = random_set(rng);
      const double xy = gospa(x, y, params).total;
      EXPECT_EQ(gospa(x, x, params).total, 0.0);
      EXPECT_NEAR(xy, gospa(y, x, params).total, 1e-9);
      EXPECT_GE(xy, 0.0);
      EXPECT_LE(gospa(x, z, params).total, xy + gospa(y, z, params).total + 1e-9);
    }
  }
}

}  // namespace
}  // namespace sifted::eval
