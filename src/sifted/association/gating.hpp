#pragma once

#include <cstddef>
#include <optional>

// Chi-square gating.
//
// For a correctly associated measurement with Gaussian errors, the squared
// Mahalanobis distance d^2 = y^T S^-1 y of its innovation is chi-square
// distributed with as many degrees of freedom as the measurement has
// components. A gate that accepts d^2 <= chi2_quantile(dof, P_G) therefore
// keeps the true measurement with probability P_G. The threshold depends on
// the measurement dimension, so a 1-D bearing and a 2-D range-bearing
// measurement are gated at the same probability.
namespace sifted::association {

inline constexpr std::size_t kMaxChiSquareDof = 100;

// Smallest x with P(chi2_dof <= x) >= p, for dof in [1, kMaxChiSquareDof] and
// p in (0, 1).
// Computed by bisection on the closed-form CDF to ~1e-12 relative precision
// (a fixed iteration count keeps it a deterministic function of its inputs).
// nullopt for unsupported dof or p outside (0, 1).
std::optional<double> chi_square_quantile(std::size_t dof, double p) noexcept;

// P(chi2_dof <= x) for dof in [1, kMaxChiSquareDof] (closed-form recurrence);
// NaN for other dof.
double chi_square_cdf(std::size_t dof, double x) noexcept;

}  // namespace sifted::association
