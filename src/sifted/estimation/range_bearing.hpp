#pragma once

#include <Eigen/Core>
#include <optional>

#include "sifted/core/sensor.hpp"
#include "sifted/estimation/cv_filter.hpp"
#include "sifted/estimation/linearized.hpp"

// Range-bearing measurement model for the extended Kalman filter.
//
// With d = target - sensor (tracking frame) and r = |d|:
//
//   h(x) = [ r,  wrap(atan2(d.y, d.x) - yaw) ]
//
//   H = dh/dx = [  d.x/r     d.y/r    0  0 ]
//               [ -d.y/r^2   d.x/r^2  0  0 ]
//
// The motion model stays the linear constant-velocity one, so "EKF" here means
// a linear predict plus this linearised update, which is exact for the motion
// and first-order for the measurement.
namespace sifted::estimation {

// Below this predicted range the bearing Jacobian is ill-conditioned (and
// undefined at 0), so updates are refused.
inline constexpr double kMinRangeForUpdateM = 1.0;

struct RangeBearing {
  double range_m = 0.0;
  double bearing_rad = 0.0;  // relative to sensor boresight, in [-pi, pi]
};

// Maps any finite angle into [-pi, pi]. Uses std::remainder, which is exact,
// so the result does not depend on the platform's libm.
double wrap_angle(double a) noexcept;

// h(x): the measurement a perfect sensor at `pose` would report.
RangeBearing predict_range_bearing(const StateVector& x, const SensorPose& pose) noexcept;

// dh/dx at `x`. Precondition: range from pose to x is at least kMinRangeForUpdateM.
Eigen::Matrix<double, 2, kStateDim> range_bearing_jacobian(const StateVector& x,
                                                           const SensorPose& pose) noexcept;

// Inverse model, used to initialise a track from one measurement: the measured
// point in the tracking frame and its covariance (first-order propagation of
// diag(sigma_r^2, sigma_b^2) through the polar-to-Cartesian conversion).
struct LocalPosition {
  Eigen::Vector2d position;
  Eigen::Matrix2d covariance;
};
LocalPosition range_bearing_to_local(const RangeBearing& z, const RangeBearing& sigma,
                                     const SensorPose& pose) noexcept;

// Innovation (bearing wrapped), Jacobian and noise for a range-bearing
// measurement `z` with 1-sigma noise `sigma`, linearised at `x`. nullopt for
// non-finite or negative input, non-positive sigmas, or a predicted range under
// kMinRangeForUpdateM.
std::optional<Linearized<2>> range_bearing_innovation(const StateVector& x, const SensorPose& pose,
                                                      const RangeBearing& z,
                                                      const RangeBearing& sigma) noexcept;

// EKF update of `filter` with a range-bearing measurement `z` whose 1-sigma
// noise is `sigma`. The bearing innovation is wrapped, so a measurement just
// across the +/-pi seam is a small correction, not a ~2*pi one.
FilterStatus update_range_bearing(ConstantVelocityFilter& filter, const SensorPose& pose,
                                  const RangeBearing& z, const RangeBearing& sigma) noexcept;

}  // namespace sifted::estimation
