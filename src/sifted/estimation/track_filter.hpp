#pragma once

#include <cstdint>
#include <optional>
#include <string_view>
#include <variant>

#include "sifted/core/result.hpp"
#include "sifted/estimation/cv_filter.hpp"
#include "sifted/estimation/imm_filter.hpp"
#include "sifted/estimation/measurement_model.hpp"

// The motion model of one track: the legacy single constant-velocity filter
// (with the engine's NIS-switched process noise, docs/design.md section 6) or
// the IMM (imm_filter.hpp, ADR 0014). The engine sees one interface:
//
//   state(), covariance()  the track estimate: the filter's own, or the IMM's
//                          combined (moment-matched) 4-state estimate;
//   components()           the Gaussian hypotheses behind it: the filter
//                          itself (1), or the IMM's mode-matched estimates;
//   gate(ctx)              the gating statistic: the smallest squared
//                          Mahalanobis distance of the measurement over the
//                          components, each linearised at its own state, i.e.
//                          the union of the components' validation regions
//                          (ADR 0014). For the constant-velocity filter it is
//                          exactly the legacy d^2;
//   mahalanobis2(l)        d^2 under state()/covariance() with `l` linearised
//                          at state(): the NIS of the track estimate;
//   update(ctx, l)         the measurement update. The constant-velocity filter
//                          applies `l` (exactly the legacy computation); the
//                          IMM linearises `ctx` again at each mode's own state.
namespace sifted::estimation {

struct GateResult {
  double d2 = 0.0;
  AnyLinearized lin;  // linearised at the component that gave d2
};

enum class MotionModel : std::uint8_t { ConstantVelocity, Imm };

std::string_view to_string(MotionModel m) noexcept;  // "cv", "imm"
std::optional<MotionModel> motion_model_from_string(std::string_view s) noexcept;

class TrackFilter {
 public:
  explicit TrackFilter(const ConstantVelocityFilter& f) : f_(f) {}
  explicit TrackFilter(const ImmFilter& f) : f_(f) {}

  MotionModel model() const noexcept {
    return f_.index() == 0 ? MotionModel::ConstantVelocity : MotionModel::Imm;
  }

  FilterStatus predict(double dt) noexcept;
  const StateVector& state() const noexcept;
  const StateCovariance& covariance() const noexcept;

  std::size_t components() const noexcept { return f_.index() == 0 ? 1U : kImmModes; }
  const StateVector& component_state(std::size_t i) const noexcept;
  const StateCovariance& component_covariance(std::size_t i) const noexcept;

  // nullopt if no component can be linearised or none gives a finite d^2.
  // Ties keep the lowest component index.
  std::optional<GateResult> gate(const MeasurementContext& ctx) const noexcept;
  std::optional<double> mahalanobis2(const AnyLinearized& l) const noexcept;
  // `l` must be `ctx` linearised at state(). All or nothing.
  FilterStatus update(const MeasurementContext& ctx, const AnyLinearized& l) noexcept;

  // Constant-velocity filter only (the legacy manoeuvre switch); false for the IMM.
  bool set_accel_psd(double accel_psd) noexcept;

  const ConstantVelocityFilter* cv() const noexcept {
    return std::get_if<ConstantVelocityFilter>(&f_);
  }
  const ImmFilter* imm() const noexcept { return std::get_if<ImmFilter>(&f_); }

 private:
  std::variant<ConstantVelocityFilter, ImmFilter> f_;
};

}  // namespace sifted::estimation
