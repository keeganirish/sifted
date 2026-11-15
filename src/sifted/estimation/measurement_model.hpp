#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <optional>
#include <variant>

#include "sifted/core/measurement.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/estimation/cv_filter.hpp"
#include "sifted/estimation/geodetic.hpp"
#include "sifted/estimation/linearized.hpp"
#include "sifted/estimation/range_bearing.hpp"

// Measurement models, linearised at a predicted state, in one shape that the
// association layer (gating) and the filter update share.
//
// For every sensor kind the model produces the innovation y = z - h(x), the
// Jacobian H = dh/dx at x and the measurement noise R. Gating uses
// S = H P H^T + R and d^2 = y^T S^-1 y; the update is the shared Joseph-form
// ConstantVelocityFilter::update_with_innovation. Bearings are always wrapped
// with wrap_angle(), so a measurement across the +/-pi seam is a small
// innovation.
//
//   kind           dim  y                                   can start a track
//   RangeBearing    2   [r - r^, wrap(b - b^)]               yes
//   BearingOnly     1   [wrap(b - b^)]                       no (range unobservable)
//   GeoPosition     2   [e - px, n - py] (tangent plane)     yes (needs a GeoOrigin)
namespace sifted::estimation {

using AnyLinearized = std::variant<Linearized<1>, Linearized<2>>;

// Dimension of a linearised measurement (1 or 2).
std::size_t dimension(const AnyLinearized& l) noexcept;

// Measurement-specific context that does not depend on the track: the sensor
// pose, and for geodetic sensors the measured point already projected onto the
// run's tangent plane. Built once per measurement.
struct MeasurementContext {
  const Measurement* m = nullptr;
  SensorPose pose{};
  // GeoPosition only: measured position in the tracking frame and its noise.
  std::optional<LocalPositionMeasurement> local;
};

// Prepares `m` for linearisation. Returns nullopt if the kind cannot be used
// in this configuration (GeoPosition without an origin, or a point the
// projection cannot represent).
std::optional<MeasurementContext> make_context(const Measurement& m, const SensorPose& pose,
                                               const std::optional<GeoOrigin>& origin) noexcept;

// y, H and R for the measurement at state `x`. nullopt if the geometry is
// degenerate (predicted range under kMinRangeForUpdateM for the angular
// models).
std::optional<AnyLinearized> linearize(const MeasurementContext& c, const StateVector& x) noexcept;

// Squared Mahalanobis distance of the innovation under the filter's current
// covariance. nullopt if S is not positive definite or inputs are non-finite.
std::optional<double> mahalanobis2(const ConstantVelocityFilter& f,
                                   const AnyLinearized& l) noexcept;

// Applies the linearised measurement to `f` (shared Joseph-form update).
FilterStatus apply(ConstantVelocityFilter& f, const AnyLinearized& l) noexcept;

// Whether a single measurement of `kind` observes a full position and can
// therefore start (and confirm) a track on its own.
constexpr bool can_initiate(SensorKind kind) noexcept {
  return kind == SensorKind::RangeBearing || kind == SensorKind::GeoPosition;
}

// Position (and its covariance) implied by one measurement, used to start a
// track. nullopt for kinds that cannot initiate.
std::optional<LocalPosition> initial_position(const MeasurementContext& c) noexcept;

}  // namespace sifted::estimation
