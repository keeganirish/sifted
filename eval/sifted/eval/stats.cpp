#include "sifted/eval/stats.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "sifted/eval/consistency.hpp"
#include "sifted/net/impairment.hpp"

namespace sifted::eval {
namespace {

void check(const std::vector<double>& x, std::size_t resamples, double level) {
  if (x.empty()) {
    throw std::invalid_argument("interval of an empty sample");
  }
  for (const double v : x) {
    if (!std::isfinite(v)) {
      throw std::invalid_argument("non-finite sample value");
    }
  }
  if (resamples == 0 || !(level > 0.0 && level < 1.0)) {
    throw std::invalid_argument("bootstrap needs resamples > 0 and level in (0, 1)");
  }
}

double mean(const std::vector<double>& x) {
  double s = 0.0;
  for (const double v : x) {
    s += v;
  }
  return s / static_cast<double>(x.size());
}

}  // namespace

std::uint64_t seed_for(std::uint64_t base, std::string_view label) noexcept {
  std::uint64_t h = 0xCBF29CE484222325ULL;  // FNV-1a 64
  for (const char c : label) {
    h ^= static_cast<unsigned char>(c);
    h *= 0x100000001B3ULL;
  }
  return net::SplitMix64(base ^ h).next();
}

double quantile(std::vector<double> x, double q) {
  if (x.empty() || !(q >= 0.0 && q <= 1.0)) {
    throw std::invalid_argument("quantile of an empty sample or q outside [0, 1]");
  }
  std::sort(x.begin(), x.end());
  const double h = q * static_cast<double>(x.size() - 1);
  const auto lo = static_cast<std::size_t>(std::floor(h));
  const std::size_t hi = std::min(lo + 1, x.size() - 1);
  return x[lo] + (h - static_cast<double>(lo)) * (x[hi] - x[lo]);
}

Estimate bootstrap_mean(const std::vector<double>& x, std::uint64_t seed, std::size_t resamples,
                        double level) {
  check(x, resamples, level);
  Estimate e;
  e.n = x.size();
  e.value = mean(x);
  net::SplitMix64 rng(seed);
  std::vector<double> means(resamples);
  for (std::size_t b = 0; b < resamples; ++b) {
    double s = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
      s += x[rng.below(x.size())];
    }
    means[b] = s / static_cast<double>(x.size());
  }
  e.lo = quantile(means, 0.5 * (1.0 - level));
  e.hi = quantile(std::move(means), 0.5 * (1.0 + level));
  return e;
}

Estimate count_mean(const std::vector<double>& counts, std::uint64_t seed, std::size_t resamples,
                    double level) {
  check(counts, resamples, level);
  if (std::all_of(counts.begin(), counts.end(), [](double v) { return v == 0.0; })) {
    // No event in n runs: the exact Poisson upper bound on the mean per run
    // (chi-square(2) quantile / 2 = -ln(alpha/2)).
    Estimate e;
    e.n = counts.size();
    e.hi = -std::log(0.5 * (1.0 - level)) / static_cast<double>(counts.size());
    e.exact_zero_bound = true;
    return e;
  }
  return bootstrap_mean(counts, seed, resamples, level);
}

Estimate paired_difference(const std::vector<double>& a, const std::vector<double>& b,
                           std::uint64_t seed, std::size_t resamples, double level) {
  if (a.size() != b.size()) {
    throw std::invalid_argument("paired difference of samples of different sizes");
  }
  std::vector<double> d(a.size());
  for (std::size_t i = 0; i < a.size(); ++i) {
    d[i] = a[i] - b[i];
  }
  return bootstrap_mean(d, seed, resamples, level);
}

Estimate wilson(std::size_t k, std::size_t n, double level) {
  if (n == 0 || k > n || !(level > 0.0 && level < 1.0)) {
    throw std::invalid_argument("Wilson interval needs 0 <= k <= n, n > 0, level in (0, 1)");
  }
  const double z = normal_quantile(0.5 * (1.0 + level));
  const double nn = static_cast<double>(n);
  const double p = static_cast<double>(k) / nn;
  const double z2 = z * z;
  const double centre = (p + z2 / (2.0 * nn)) / (1.0 + z2 / nn);
  const double half = z / (1.0 + z2 / nn) * std::sqrt(p * (1.0 - p) / nn + z2 / (4.0 * nn * nn));
  Estimate e;
  e.n = n;
  e.value = p;
  e.lo = k == 0 ? 0.0 : std::max(0.0, centre - half);
  e.hi = k == n ? 1.0 : std::min(1.0, centre + half);
  return e;
}

}  // namespace sifted::eval
