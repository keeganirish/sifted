#include "sifted/estimation/cv_filter.hpp"

#include <cmath>

namespace sifted::estimation {

std::string_view to_string(FilterStatus s) noexcept {
  switch (s) {
    case FilterStatus::Ok:
      return "ok";
    case FilterStatus::InvalidTimeStep:
      return "invalid time step";
    case FilterStatus::InvalidInput:
      return "invalid input";
    case FilterStatus::NotPositiveDefinite:
      return "innovation covariance not positive definite";
    case FilterStatus::NumericalFailure:
      return "numerical failure";
  }
  return "unknown status";
}

Result<ConstantVelocityFilter, FilterStatus> ConstantVelocityFilter::create(
    const StateVector& x0, const StateCovariance& P0, double accel_psd) {
  if (!x0.allFinite() || !is_covariance(P0) || !(accel_psd >= 0.0) || !std::isfinite(accel_psd)) {
    return FilterStatus::InvalidInput;
  }
  return ConstantVelocityFilter(x0, P0, accel_psd);
}

StateCovariance ConstantVelocityFilter::transition(double dt) noexcept {
  StateCovariance F = StateCovariance::Identity();
  F(0, 2) = dt;
  F(1, 3) = dt;
  return F;
}

StateCovariance ConstantVelocityFilter::process_noise(double dt, double accel_psd) noexcept {
  const double dt2 = dt * dt;
  const double q_pp = accel_psd * dt2 * dt / 3.0;
  const double q_pv = accel_psd * dt2 / 2.0;
  const double q_vv = accel_psd * dt;
  StateCovariance Q = StateCovariance::Zero();
  for (int axis = 0; axis < 2; ++axis) {
    const int p = axis;
    const int v = axis + 2;
    Q(p, p) = q_pp;
    Q(p, v) = q_pv;
    Q(v, p) = q_pv;
    Q(v, v) = q_vv;
  }
  return Q;
}

FilterStatus ConstantVelocityFilter::predict(double dt) noexcept {
  // `!(dt >= 0)` also catches NaN.
  if (!(dt >= 0.0) || !(dt <= kMaxPredictDt)) {
    return FilterStatus::InvalidTimeStep;
  }
  if (dt == 0.0) {
    return FilterStatus::Ok;  // exact no-op keeps replays of same-time input bit-identical
  }
  const StateCovariance F = transition(dt);
  const StateVector x_new = F * x_;
  StateCovariance P_new = F * P_ * F.transpose() + process_noise(dt, accel_psd_);
  P_new = 0.5 * (P_new + P_new.transpose());
  if (!x_new.allFinite() || !P_new.allFinite()) {
    return FilterStatus::NumericalFailure;
  }
  x_ = x_new;
  P_ = P_new;
  return FilterStatus::Ok;
}

FilterStatus ConstantVelocityFilter::update(const Eigen::Vector2d& z,
                                            const Eigen::Matrix2d& R) noexcept {
  if (!z.allFinite()) {
    return FilterStatus::InvalidInput;
  }
  Eigen::Matrix<double, 2, kStateDim> H = Eigen::Matrix<double, 2, kStateDim>::Zero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;
  const Eigen::Vector2d y = z - H * x_;
  return update_with_innovation<2>(y, H, R);
}

}  // namespace sifted::estimation
