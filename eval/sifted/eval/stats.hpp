#pragma once

// Interval estimates over Monte Carlo runs (docs/evaluation-monte-carlo.md,
// ADR 0015). The run is the independent unit: every function takes one value
// per run, never time-correlated samples from within a run.
//
//   bootstrap_mean     seeded percentile bootstrap of the mean (Efron and
//                      Tibshirani, 1993, ch. 13). No normality assumption, so
//                      it suits skewed per-run counts (ID switches, false
//                      tracks) as well as RMSE and GOSPA.
//   count_mean         bootstrap_mean for non-negative counts, except that a
//                      sample with no event at all gets the exact Poisson
//                      bound [0, -ln((1 - level) / 2) / n] instead of the
//                      degenerate [0, 0] a bootstrap would give.
//   paired_difference  bootstrap of mean(a_i - b_i) over runs i that share
//                      their random numbers (same noise realisation): the
//                      common-random-numbers comparison of two trackers or
//                      two conditions.
//   wilson             Wilson score interval for a proportion k / n (sensible
//                      at k = 0 and k = n, unlike the normal approximation).
//
// Determinism: the bootstrap draws from SplitMix64 seeded by the caller
// (derive it with seed_for), sums in index order, and sorts; the same inputs
// and seed give the same bytes on the same binary and architecture.

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace sifted::eval {

inline constexpr std::size_t kBootstrapResamples = 2000;
inline constexpr double kIntervalLevel = 0.95;

struct Estimate {
  double value = 0.0;  // point estimate (the sample mean, or k / n)
  double lo = 0.0;
  double hi = 0.0;
  std::size_t n = 0;              // runs (or trials) it is computed from
  bool exact_zero_bound = false;  // count_mean with no event: Poisson bound
};

// A 64-bit seed for one estimate: SplitMix64 of base ^ FNV-1a(label), so
// every (condition, metric) pair has its own stream whatever the order in
// which estimates are computed.
std::uint64_t seed_for(std::uint64_t base, std::string_view label) noexcept;

// Linear-interpolation quantile (Hyndman and Fan type 7) of `x`, q in [0, 1].
// Throws std::invalid_argument if x is empty or q is out of range.
double quantile(std::vector<double> x, double q);

// Throws std::invalid_argument if x is empty, a value is not finite, B is 0
// or level is not in (0, 1).
Estimate bootstrap_mean(const std::vector<double>& x, std::uint64_t seed,
                        std::size_t resamples = kBootstrapResamples, double level = kIntervalLevel);
Estimate count_mean(const std::vector<double>& counts, std::uint64_t seed,
                    std::size_t resamples = kBootstrapResamples, double level = kIntervalLevel);
// a and b must have the same size (paired by index).
Estimate paired_difference(const std::vector<double>& a, const std::vector<double>& b,
                           std::uint64_t seed, std::size_t resamples = kBootstrapResamples,
                           double level = kIntervalLevel);
// Throws std::invalid_argument if n is 0 or k > n.
Estimate wilson(std::size_t k, std::size_t n, double level = kIntervalLevel);

}  // namespace sifted::eval
