// Interval estimates for the Monte Carlo evaluation
// (eval/sifted/eval/stats.hpp): known samples, zero-event and all-event
// cases, paired differences, and deterministic output for a fixed seed.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "sifted/eval/stats.hpp"
#include "sifted/net/impairment.hpp"

namespace sifted::eval {
namespace {

TEST(MonteCarloStats, SeedDerivationIsStable) {
  // Golden values from an independent implementation (SplitMix64 of
  // base ^ FNV-1a-64(label)): changing them changes every published interval.
  EXPECT_EQ(seed_for(1, "close_pass/id_switches"), 0x1bb54a13740f742eULL);
  EXPECT_EQ(seed_for(0, ""), 0xc3817c016ba4ff30ULL);
  EXPECT_NE(seed_for(1, "a"), seed_for(1, "b"));
  EXPECT_NE(seed_for(1, "a"), seed_for(2, "a"));
}

TEST(MonteCarloStats, QuantileInterpolatesLinearly) {
  const std::vector<double> x{4, 1, 3, 2};
  EXPECT_DOUBLE_EQ(quantile(x, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(quantile(x, 0.25), 1.75);
  EXPECT_DOUBLE_EQ(quantile(x, 0.5), 2.5);
  EXPECT_DOUBLE_EQ(quantile(x, 1.0), 4.0);
  EXPECT_DOUBLE_EQ(quantile({7}, 0.3), 7.0);
  EXPECT_THROW(quantile({}, 0.5), std::invalid_argument);
  EXPECT_THROW(quantile(x, 1.5), std::invalid_argument);
}

TEST(MonteCarloStats, BootstrapOfAConstantIsThatConstant) {
  const Estimate e = bootstrap_mean({5, 5, 5}, 1);
  EXPECT_EQ(e.value, 5.0);
  EXPECT_EQ(e.lo, 5.0);
  EXPECT_EQ(e.hi, 5.0);
  EXPECT_EQ(e.n, 3U);
}

TEST(MonteCarloStats, BootstrapIntervalMatchesTheKnownStandardError) {
  std::vector<double> x;
  for (int i = 0; i < 100; ++i) {
    x.push_back(i);
  }
  // Mean 49.5, sd 29.01, se 2.90: a 95% interval of about +/- 5.7.
  const Estimate e = bootstrap_mean(x, 42);
  EXPECT_DOUBLE_EQ(e.value, 49.5);
  EXPECT_GT(e.lo, 42.5);
  EXPECT_LT(e.lo, 45.0);
  EXPECT_GT(e.hi, 54.0);
  EXPECT_LT(e.hi, 56.5);
}

TEST(MonteCarloStats, BootstrapIsDeterministicForASeed) {
  const std::vector<double> x{0, 0, 1, 0, 3, 0, 0, 7, 0, 1};
  const Estimate a = bootstrap_mean(x, 7);
  const Estimate b = bootstrap_mean(x, 7);
  EXPECT_EQ(a.lo, b.lo);
  EXPECT_EQ(a.hi, b.hi);
  const Estimate c = bootstrap_mean(x, 8);
  EXPECT_TRUE(a.lo != c.lo || a.hi != c.hi);
  // Skewed counts: the interval is asymmetric around the mean.
  EXPECT_LT(a.value - a.lo, a.hi - a.value);
}

TEST(MonteCarloStats, CountWithNoEventGetsTheExactPoissonBound) {
  const Estimate e = count_mean(std::vector<double>(100, 0.0), 1);
  EXPECT_TRUE(e.exact_zero_bound);
  EXPECT_EQ(e.value, 0.0);
  EXPECT_EQ(e.lo, 0.0);
  EXPECT_NEAR(e.hi, -std::log(0.025) / 100.0, 1e-15);  // 0.0369 per run
  const Estimate some = count_mean({0, 0, 2}, 1);
  EXPECT_FALSE(some.exact_zero_bound);
  EXPECT_GT(some.hi, 0.0);
}

TEST(MonteCarloStats, PairedDifferenceRemovesTheCommonNoise) {
  net::SplitMix64 rng(3);
  std::vector<double> a;
  std::vector<double> b;
  for (int i = 0; i < 200; ++i) {
    const double common = 100.0 * rng.uniform();  // large, shared run effect
    a.push_back(common + 1.0 + 0.1 * rng.uniform());
    b.push_back(common);
  }
  const Estimate d = paired_difference(a, b, 11);
  EXPECT_NEAR(d.value, 1.05, 0.01);
  EXPECT_GT(d.lo, 1.0);  // the shared noise does not widen the interval
  EXPECT_LT(d.hi, 1.1);
  EXPECT_THROW(paired_difference({1, 2}, {1}, 1), std::invalid_argument);
  const Estimate same = paired_difference({1, 2, 3}, {1, 2, 3}, 1);
  EXPECT_EQ(same.lo, 0.0);
  EXPECT_EQ(same.hi, 0.0);
}

TEST(MonteCarloStats, WilsonIntervalOnKnownProportions) {
  const Estimate none = wilson(0, 10);
  EXPECT_EQ(none.lo, 0.0);
  EXPECT_NEAR(none.hi, 0.2775, 1e-4);  // z^2 / (n + z^2)
  const Estimate half = wilson(5, 10);
  EXPECT_NEAR(half.lo, 0.2366, 1e-4);
  EXPECT_NEAR(half.hi, 0.7634, 1e-4);
  const Estimate all = wilson(10, 10);
  EXPECT_EQ(all.hi, 1.0);
  EXPECT_NEAR(all.lo, 1.0 - 0.2775, 1e-4);
  EXPECT_THROW(wilson(0, 0), std::invalid_argument);
  EXPECT_THROW(wilson(3, 2), std::invalid_argument);
}

TEST(MonteCarloStats, RejectsBadSamples) {
  EXPECT_THROW(bootstrap_mean({}, 1), std::invalid_argument);
  EXPECT_THROW(bootstrap_mean({1.0, std::numeric_limits<double>::quiet_NaN()}, 1),
               std::invalid_argument);
  EXPECT_THROW(bootstrap_mean({1.0}, 1, 0), std::invalid_argument);
  EXPECT_THROW(bootstrap_mean({1.0}, 1, 10, 1.0), std::invalid_argument);
}

}  // namespace
}  // namespace sifted::eval
