#include "sifted/estimation/bearing_only.hpp"

#include <cmath>

#include "sifted/estimation/range_bearing.hpp"

namespace sifted::estimation {

double predict_bearing(const StateVector& x, const SensorPose& pose) noexcept {
  return wrap_angle(std::atan2(x(1) - pose.y_m, x(0) - pose.x_m) - pose.yaw_rad);
}

Eigen::Matrix<double, 1, kStateDim> bearing_jacobian(const StateVector& x,
                                                     const SensorPose& pose) noexcept {
  const double dx = x(0) - pose.x_m;
  const double dy = x(1) - pose.y_m;
  const double r2 = dx * dx + dy * dy;
  Eigen::Matrix<double, 1, kStateDim> H = Eigen::Matrix<double, 1, kStateDim>::Zero();
  H(0, 0) = -dy / r2;
  H(0, 1) = dx / r2;
  return H;
}

std::optional<Linearized<1>> bearing_innovation(const StateVector& x, const SensorPose& pose,
                                                double z, double sigma) noexcept {
  if (!std::isfinite(z) || !(sigma > 0.0) || !std::isfinite(sigma) || !x.allFinite()) {
    return std::nullopt;
  }
  const double dx = x(0) - pose.x_m;
  const double dy = x(1) - pose.y_m;
  if (!(std::sqrt(dx * dx + dy * dy) >= kMinRangeForUpdateM)) {
    return std::nullopt;
  }
  Linearized<1> l;
  l.y(0) = wrap_angle(z - predict_bearing(x, pose));
  l.H = bearing_jacobian(x, pose);
  l.R(0, 0) = sigma * sigma;
  return l;
}

FilterStatus update_bearing_only(ConstantVelocityFilter& filter, const SensorPose& pose, double z,
                                 double sigma) noexcept {
  const auto l = bearing_innovation(filter.state(), pose, z, sigma);
  if (!l) {
    return FilterStatus::InvalidInput;
  }
  return filter.update_with_innovation<1>(l->y, l->H, l->R);
}

}  // namespace sifted::estimation
