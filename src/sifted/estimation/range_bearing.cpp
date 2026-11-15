#include "sifted/estimation/range_bearing.hpp"

#include <cmath>

namespace sifted::estimation {
namespace {

constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

}  // namespace

double wrap_angle(double a) noexcept { return std::remainder(a, kTwoPi); }

RangeBearing predict_range_bearing(const StateVector& x, const SensorPose& pose) noexcept {
  const double dx = x(0) - pose.x_m;
  const double dy = x(1) - pose.y_m;
  return {std::sqrt(dx * dx + dy * dy), wrap_angle(std::atan2(dy, dx) - pose.yaw_rad)};
}

Eigen::Matrix<double, 2, kStateDim> range_bearing_jacobian(const StateVector& x,
                                                           const SensorPose& pose) noexcept {
  const double dx = x(0) - pose.x_m;
  const double dy = x(1) - pose.y_m;
  const double r2 = dx * dx + dy * dy;
  const double r = std::sqrt(r2);
  Eigen::Matrix<double, 2, kStateDim> H = Eigen::Matrix<double, 2, kStateDim>::Zero();
  H(0, 0) = dx / r;
  H(0, 1) = dy / r;
  H(1, 0) = -dy / r2;
  H(1, 1) = dx / r2;
  return H;
}

LocalPosition range_bearing_to_local(const RangeBearing& z, const RangeBearing& sigma,
                                     const SensorPose& pose) noexcept {
  const double theta = z.bearing_rad + pose.yaw_rad;  // angle in the tracking frame
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  LocalPosition out;
  out.position = Eigen::Vector2d(pose.x_m + z.range_m * c, pose.y_m + z.range_m * s);

  Eigen::Matrix2d J;  // d(position)/d(range, bearing)
  J(0, 0) = c;
  J(0, 1) = -z.range_m * s;
  J(1, 0) = s;
  J(1, 1) = z.range_m * c;
  Eigen::Matrix2d Rz = Eigen::Matrix2d::Zero();
  Rz(0, 0) = sigma.range_m * sigma.range_m;
  Rz(1, 1) = sigma.bearing_rad * sigma.bearing_rad;
  out.covariance = J * Rz * J.transpose();
  out.covariance = 0.5 * (out.covariance + out.covariance.transpose());
  return out;
}

FilterStatus update_range_bearing(ConstantVelocityFilter& filter, const SensorPose& pose,
                                  const RangeBearing& z, const RangeBearing& sigma) noexcept {
  if (!std::isfinite(z.range_m) || !std::isfinite(z.bearing_rad) || !(z.range_m >= 0.0) ||
      !(sigma.range_m > 0.0) || !(sigma.bearing_rad > 0.0) || !std::isfinite(sigma.range_m) ||
      !std::isfinite(sigma.bearing_rad)) {
    return FilterStatus::InvalidInput;
  }
  const StateVector& x = filter.state();
  const RangeBearing predicted = predict_range_bearing(x, pose);
  if (!(predicted.range_m >= kMinRangeForUpdateM)) {
    return FilterStatus::InvalidInput;
  }

  Eigen::Vector2d y;
  y(0) = z.range_m - predicted.range_m;
  y(1) = wrap_angle(z.bearing_rad - predicted.bearing_rad);

  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = sigma.range_m * sigma.range_m;
  R(1, 1) = sigma.bearing_rad * sigma.bearing_rad;

  return filter.update_with_innovation<2>(y, range_bearing_jacobian(x, pose), R);
}

}  // namespace sifted::estimation
