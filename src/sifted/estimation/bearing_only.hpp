#pragma once

#include <Eigen/Core>
#include <optional>

#include "sifted/core/sensor.hpp"
#include "sifted/estimation/cv_filter.hpp"
#include "sifted/estimation/linearized.hpp"

// Bearing-only (camera-like) measurement model for the EKF.
//
// With d = target - sensor (tracking frame) and r^2 = |d|^2:
//
//   h(x) = wrap(atan2(d.y, d.x) - yaw)
//   H    = dh/dx = [ -d.y/r^2   d.x/r^2   0   0 ]
//
// It is the bearing row of the range-bearing model. One bearing constrains the
// target to a ray, so range is unobservable from a single measurement: a
// bearing-only measurement can update an existing track but never start one
// (docs/design.md section 6, ADR 0004). Range becomes observable only by fusing
// with a position-observing sensor or through geometry (crossing bearings from
// separated sensors).
namespace sifted::estimation {

// h(x), wrapped into [-pi, pi].
double predict_bearing(const StateVector& x, const SensorPose& pose) noexcept;

// dh/dx at `x`. Precondition: range from pose to x is at least kMinRangeForUpdateM.
Eigen::Matrix<double, 1, kStateDim> bearing_jacobian(const StateVector& x,
                                                     const SensorPose& pose) noexcept;

// Innovation (wrapped), Jacobian and noise for a bearing `z` with 1-sigma
// `sigma`. nullopt for non-finite input, a non-positive sigma, or a predicted
// range under kMinRangeForUpdateM (the Jacobian is singular at the sensor).
std::optional<Linearized<1>> bearing_innovation(const StateVector& x, const SensorPose& pose,
                                                double z, double sigma) noexcept;

// EKF update of `filter` with one bearing measurement, through the shared
// Joseph-form update. State is unchanged on any failure.
FilterStatus update_bearing_only(ConstantVelocityFilter& filter, const SensorPose& pose, double z,
                                 double sigma) noexcept;

}  // namespace sifted::estimation
