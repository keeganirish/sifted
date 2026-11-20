#include "sifted/estimation/imm_filter.hpp"

#include <algorithm>
#include <cmath>

namespace sifted::estimation {

bool is_valid(const ImmParams& p) noexcept {
  for (std::size_t i = 0; i < kImmModes; ++i) {
    const double q = p.accel_psd[i];
    const double tau = p.sojourn_s[i];
    if (!(q >= 0.0) || !std::isfinite(q) || (i > 0 && !(q >= p.accel_psd[i - 1]))) {
      return false;
    }
    if (!(tau >= kMinSojournS && tau <= kMaxSojournS)) {
      return false;
    }
  }
  return p.probability_floor > 0.0 &&
         p.probability_floor <= 1.0 / (2.0 * static_cast<double>(kImmModes));
}

ModeMatrix transition_matrix(const ImmParams& p, double dt) noexcept {
  if (!(dt > 0.0)) {
    return ModeMatrix::Identity();
  }
  constexpr auto n = static_cast<Eigen::Index>(kImmModes);
  ModeMatrix A;  // G dt
  for (Eigen::Index i = 0; i < n; ++i) {
    const double rate = dt / p.sojourn_s[static_cast<std::size_t>(i)];
    for (Eigen::Index j = 0; j < n; ++j) {
      A(i, j) = i == j ? -rate : rate / static_cast<double>(kImmModes - 1);
    }
  }
  // ||A||_inf = 2 dt / min tau. Scale by 2^-s (exact) until it is <= 1/2,
  // where the degree-12 Taylor remainder is below 1e-16 relative.
  const double norm = 2.0 * dt / *std::min_element(p.sojourn_s.begin(), p.sojourn_s.end());
  int s = 0;
  if (norm > 0.5) {
    int e = 0;
    (void)std::frexp(norm / 0.5, &e);  // norm / 0.5 = f 2^e, f in [0.5, 1)
    s = e;
  }
  const ModeMatrix B = std::ldexp(1.0, -s) * A;
  ModeMatrix E = ModeMatrix::Identity();
  ModeMatrix term = ModeMatrix::Identity();
  for (int k = 1; k <= 12; ++k) {
    term = (term * B) / static_cast<double>(k);
    E += term;
  }
  for (int k = 0; k < s; ++k) {
    E = (E * E).eval();
  }
  for (Eigen::Index i = 0; i < n; ++i) {
    for (Eigen::Index j = 0; j < n; ++j) {
      E(i, j) = std::max(E(i, j), 0.0);  // rounding can leave -1e-17
    }
    E.row(i) /= E.row(i).sum();
  }
  return E;
}

ModeVector stationary_probabilities(const ImmParams& p) noexcept {
  ModeVector pi;
  double sum = 0.0;
  for (std::size_t i = 0; i < kImmModes; ++i) {
    sum += p.sojourn_s[i];
  }
  for (std::size_t i = 0; i < kImmModes; ++i) {
    pi(static_cast<Eigen::Index>(i)) = p.sojourn_s[i] / sum;
  }
  return pi;
}

ImmFilter::ImmFilter(const StateVector& x0, const StateCovariance& P0, const ImmParams& params)
    : mu_(stationary_probabilities(params)), x_(x0), P_(P0), params_(params) {
  for (ModeEstimate& m : modes_) {
    m = {x0, P0};
  }
}

Result<ImmFilter, FilterStatus> ImmFilter::create(const StateVector& x0, const StateCovariance& P0,
                                                  const ImmParams& params) {
  if (!x0.allFinite() || !is_covariance(P0) || !is_valid(params)) {
    return FilterStatus::InvalidInput;
  }
  return ImmFilter(x0, P0, params);
}

void ImmFilter::combine(const std::array<ModeEstimate, kImmModes>& modes, const ModeVector& mu,
                        StateVector& x, StateCovariance& P) noexcept {
  x = StateVector::Zero();
  for (std::size_t j = 0; j < kImmModes; ++j) {
    x += mu(static_cast<Eigen::Index>(j)) * modes[j].x;
  }
  P = StateCovariance::Zero();
  for (std::size_t j = 0; j < kImmModes; ++j) {
    const StateVector d = modes[j].x - x;
    P += mu(static_cast<Eigen::Index>(j)) * (modes[j].P + d * d.transpose());
  }
  P = (0.5 * (P + P.transpose())).eval();  // eval: exactly symmetric, no aliasing
}

ModeVector ImmFilter::apply_floor(const ModeVector& mu, double floor) noexcept {
  const ModeVector m = mu.cwiseMax(floor);
  return m / m.sum();
}

FilterStatus ImmFilter::predict(double dt) noexcept {
  // `!(dt >= 0)` also catches NaN.
  if (!(dt >= 0.0) || !(dt <= ConstantVelocityFilter::kMaxPredictDt)) {
    return FilterStatus::InvalidTimeStep;
  }
  if (dt == 0.0) {
    return FilterStatus::Ok;  // exact no-op: no time, no transition, no motion
  }
  constexpr auto n = static_cast<Eigen::Index>(kImmModes);
  const ModeMatrix Pi = transition_matrix(params_, dt);
  const ModeVector c = Pi.transpose() * mu_;  // c_j = sum_i Pi_ij mu_i
  const StateCovariance F = ConstantVelocityFilter::transition(dt);
  std::array<ModeEstimate, kImmModes> out;
  for (Eigen::Index j = 0; j < n; ++j) {
    if (!(c(j) > 0.0) || !std::isfinite(c(j))) {
      return FilterStatus::NumericalFailure;
    }
    // Mixing: mu_i|j = Pi_ij mu_i / c_j.
    ModeVector w;
    for (Eigen::Index i = 0; i < n; ++i) {
      w(i) = Pi(i, j) * mu_(i) / c(j);
    }
    StateVector x0;
    StateCovariance P0;
    combine(modes_, w, x0, P0);
    const auto jj = static_cast<std::size_t>(j);
    out[jj].x = F * x0;
    StateCovariance P =
        F * P0 * F.transpose() + ConstantVelocityFilter::process_noise(dt, params_.accel_psd[jj]);
    out[jj].P = 0.5 * (P + P.transpose());
    if (!out[jj].x.allFinite() || !out[jj].P.allFinite()) {
      return FilterStatus::NumericalFailure;
    }
  }
  const ModeVector mu = c / c.sum();
  StateVector x;
  StateCovariance P;
  combine(out, mu, x, P);
  if (!x.allFinite() || !P.allFinite()) {
    return FilterStatus::NumericalFailure;
  }
  modes_ = out;
  mu_ = mu;
  x_ = x;
  P_ = P;
  return FilterStatus::Ok;
}

}  // namespace sifted::estimation
