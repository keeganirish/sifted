#pragma once

// Filter-consistency statistics for the ground-truth evaluation
// (Bar-Shalom, Li and Kirubarajan, "Estimation with Applications to Tracking
// and Navigation", 2001, sec. 5.4).
//
//   NEES  e^T P^-1 e, e = estimate - truth: for a consistent filter it is
//         chi-square with dim(e) degrees of freedom, mean dim(e).
//   NIS   y^T S^-1 y of an associated measurement: chi-square with the
//         measurement dimension, mean = that dimension.
//
// The average of N samples whose degrees of freedom sum to K is, if the
// samples are independent, (chi-square with K dof) / N. The acceptance
// interval below is derived from that distribution, never chosen.
//
// Caveat, and it matters: samples are NOT independent. Consecutive NEES
// samples of one track share most of their error (the filter's memory spans
// several updates), so the effective sample count is much smaller than N and
// the true spread of the average is wider than the interval says. The interval
// is a guide, not an exact test. Innovations of a consistent filter are white,
// so the NIS interval is much closer to exact than the NEES one; it is still
// not exact here because gating truncates the tail (0.1% per update) and
// clutter can be associated.

#include <Eigen/Core>
#include <cstddef>

namespace sifted::eval {

// e^T P^-1 e (P symmetric positive definite; NaN if it is not).
double nees(const Eigen::Ref<const Eigen::VectorXd>& e, const Eigen::Ref<const Eigen::MatrixXd>& P);

// Standard normal quantile, p in (0, 1) (bisection on erfc; NaN outside).
double normal_quantile(double p);

struct Interval {
  double lo = 0.0;
  double hi = 0.0;
  bool contains(double v) const noexcept { return v >= lo && v <= hi; }
};

// Two-sided interval holding a chi-square(dof) variable with probability
// `probability`. Exact quantiles (association::chi_square_quantile) up to
// association::kMaxChiSquareDof; beyond it the Wilson-Hilferty cube-root
// normal approximation (relative error below 1e-3 for dof > 100).
Interval chi_square_interval(std::size_t dof, double probability = 0.95);

// Sum, count and total degrees of freedom of chi-square samples.
struct ChiSquareStat {
  double sum = 0.0;
  std::size_t n = 0;
  std::size_t dof = 0;  // sum of the samples' degrees of freedom

  void add(double value, std::size_t sample_dof) {
    sum += value;
    ++n;
    dof += sample_dof;
  }
  void merge(const ChiSquareStat& o) {
    sum += o.sum;
    n += o.n;
    dof += o.dof;
  }
  double average() const noexcept { return n ? sum / static_cast<double>(n) : 0.0; }
  // What the average should be for a consistent filter (mean dof per sample).
  double expected() const noexcept {
    return n ? static_cast<double>(dof) / static_cast<double>(n) : 0.0;
  }
  // Acceptance interval for average() under independence (see the caveat).
  Interval interval(double probability = 0.95) const;
};

}  // namespace sifted::eval
