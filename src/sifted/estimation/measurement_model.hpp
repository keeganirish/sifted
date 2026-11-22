#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
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

// Gate screen (performance; docs/design.md section 10, ADR 0011): a cheap
// test, per (measurement, track) pair, that is false only when the pair's
// squared Mahalanobis distance is certainly greater than `limit`, so
// linearize() and mahalanobis2() -- an atan2, a Jacobian, H P H^T and a
// Cholesky per pair -- can be skipped. It never rejects a pair that could
// gate, so association results are bit for bit those of the full
// computation (checked by tests and by the engine fuzz target).
//
// With D the plane distance between the measured point and the track's
// predicted position p, lambda the largest eigenvalue of the predicted
// position covariance, and S = H P H^T + R:
//   * for any SPD S, d^2 = y'S^-1 y >= y_i^2 / S_ii (Cauchy-Schwarz);
//   * range-bearing: S_rr <= lambda + sigma_r^2, S_bb <= lambda / r^2 +
//     sigma_b^2 (r: predicted range; |dh/dp| is 1 and 1/r), and
//     D^2 = y_r^2 + 4 r_m r sin^2(y_b / 2) <= y_r^2 + r_m r y_b^2.
//     So d^2 <= limit implies D^2 <= limit * (lambda (1 + r_m / r) +
//     sigma_r^2 + r_m r sigma_b^2);
//   * position (geodetic): S = P_pos + R, so d^2 >= D^2 / (lambda +
//     lambda_max(R));
//   * bearing-only: no screen (it would need the atan2 it is meant to save).
// The bound is widened by a relative 1e-6 so floating-point rounding in the
// full computation can never put a screened-out pair inside the limit.
//
// A track may carry several Gaussian components (the IMM's mode-matched
// predictions (x_j, P_j), ADR 0014). Its gating statistic is the mixture
// statistic d2_mix = -2 ln sum_j w_j exp(-d_j^2 / 2) with weights w_j >= 0
// summing to 1 (estimation::TrackFilter::gate), and every d_j^2 is the
// single-Gaussian distance above, computed with that mode's own position and
// covariance. Since sum_j w_j exp(-d_j^2 / 2) <= exp(-min_j d_j^2 / 2),
// d2_mix >= min_j d_j^2 (and the implementation clamps it to that bound, so
// rounding cannot break it). Hence d2_mix <= limit implies d_j^2 <= limit for
// some j, which by the argument above implies may_gate(p_j, lambda_j). The
// engine therefore may skip a pair only if may_gate is false for every
// component, which it tests with one envelope (may_gate_envelope, below):
// provably conservative for the mixture. A single-component track still uses
// may_gate itself, exactly the single-filter screen.
struct GateScreen {
  enum class Kind : std::uint8_t { None, RangeBearing, Position };
  Kind kind = Kind::None;
  Eigen::Vector2d point = Eigen::Vector2d::Zero();  // measured position in the tracking frame
  Eigen::Vector2d sensor = Eigen::Vector2d::Zero();
  double range = 0.0;      // r_m
  double var_range = 0.0;  // sigma_r^2 (range-bearing) or lambda_max(R) (position)
  double var_bearing = 0.0;
};
GateScreen make_screen(const MeasurementContext& c) noexcept;
// `position`: predicted track position; `lambda`: largest eigenvalue of its
// 2x2 position covariance. True: compute the pair.
bool may_gate(const GateScreen& s, const Eigen::Vector2d& position, double lambda,
              double limit) noexcept;

// One test for every component of a mixture at once (ADR 0014). An envelope
// covers every component whose position lies within `spread` of `center` and
// whose largest position variance is at most `lambda`; it is built once per
// track and group, from the group's sensor position. may_gate_envelope is
// false only if may_gate is false for every such component. Proof: such a
// component's plane distance to the measured point is at least
// D_center - spread; its range from the sensor lies in
// [r_center - spread, r_center + spread]; may_gate's bound B grows with lambda
// and is convex in that range, so it is largest at an end of the interval.
// The final comparison avoids a square root by testing
// D_center^2 > 2 B + 2 spread^2, which implies D_center - spread > sqrt(B).
// The engine uses it for IMM tracks (center: combined position; spread:
// largest distance of a mode from it; lambda: largest mode variance), so the
// screen costs one cheap test per pair whatever the number of modes.
struct GateEnvelope {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  Eigen::Vector2d sensor = Eigen::Vector2d::Zero();
  double spread = 0.0;
  double lambda = 0.0;
  double r_lo = 0.0;  // range interval of the components from `sensor`
  double r_hi = 0.0;
  bool valid = false;  // false: never screen
};
GateEnvelope make_envelope(const Eigen::Vector2d& center, double spread, double lambda,
                           const Eigen::Vector2d& sensor) noexcept;
bool may_gate_envelope(const GateScreen& s, const GateEnvelope& e, double limit) noexcept;

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
