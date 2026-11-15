#include "sifted/association/gating.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sifted::association {
namespace {
constexpr double kPi = 3.14159265358979323846;
}  // namespace

double chi_square_cdf(std::size_t dof, double x) noexcept {
  if (dof < 1 || dof > kMaxChiSquareDof) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (!(x > 0.0)) {
    return x == 0.0 ? 0.0 : std::numeric_limits<double>::quiet_NaN();
  }
  // Regularised lower incomplete gamma P(dof/2, x/2), built up from the
  // closed forms for dof 1 and 2 with P(a + 1, h) = P(a, h) - h^a e^-h / Gamma(a + 1).
  const double h = 0.5 * x;
  double a = dof % 2 == 1 ? 0.5 : 1.0;
  double p = dof % 2 == 1 ? std::erf(std::sqrt(h)) : -std::expm1(-h);
  // term = h^a e^-h / Gamma(a + 1), updated multiplicatively.
  double term = dof % 2 == 1 ? std::sqrt(h / kPi) * 2.0 * std::exp(-h) : h * std::exp(-h);
  while (2.0 * a < static_cast<double>(dof)) {
    p -= term;
    a += 1.0;
    term *= h / (a);
  }
  return std::max(0.0, p);
}

std::optional<double> chi_square_quantile(std::size_t dof, double p) noexcept {
  if (dof < 1 || dof > kMaxChiSquareDof || !(p > 0.0 && p < 1.0)) {
    return std::nullopt;
  }
  double lo = 0.0;
  double hi = 1.0;
  while (chi_square_cdf(dof, hi) < p) {
    hi *= 2.0;
    if (hi > 1e6) {
      return std::nullopt;  // p too close to 1 to represent
    }
  }
  for (int i = 0; i < 200 && hi - lo > 1e-12 * hi; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (chi_square_cdf(dof, mid) < p) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return hi;
}

}  // namespace sifted::association
