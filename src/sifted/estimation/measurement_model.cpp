#include "sifted/estimation/measurement_model.hpp"

#include <cmath>

#include "sifted/estimation/bearing_only.hpp"
#include "sifted/estimation/range_bearing.hpp"

namespace sifted::estimation {

std::size_t dimension(const AnyLinearized& l) noexcept { return l.index() == 0 ? 1U : 2U; }

std::optional<MeasurementContext> make_context(const Measurement& m, const SensorPose& pose,
                                               const std::optional<GeoOrigin>& origin) noexcept {
  MeasurementContext c;
  c.m = &m;
  c.pose = pose;
  if (m.kind == SensorKind::GeoPosition) {
    if (!origin) {
      return std::nullopt;
    }
    c.local = geodetic_measurement_to_local(*origin, {m.z[0], m.z[1], m.z[2]},
                                            {m.sigma[0], m.sigma[1], m.sigma[2]});
    if (!c.local) {
      return std::nullopt;
    }
  }
  return c;
}

std::optional<AnyLinearized> linearize(const MeasurementContext& c, const StateVector& x) noexcept {
  const Measurement& m = *c.m;
  switch (m.kind) {
    case SensorKind::RangeBearing: {
      auto l = range_bearing_innovation(x, c.pose, {m.z[0], m.z[1]}, {m.sigma[0], m.sigma[1]});
      if (!l) {
        return std::nullopt;
      }
      return AnyLinearized{*l};
    }
    case SensorKind::BearingOnly: {
      auto l = bearing_innovation(x, c.pose, m.z[0], m.sigma[0]);
      if (!l) {
        return std::nullopt;
      }
      return AnyLinearized{*l};
    }
    case SensorKind::GeoPosition: {
      if (!c.local) {
        return std::nullopt;
      }
      Linearized<2> l;
      l.H = Eigen::Matrix<double, 2, kStateDim>::Zero();
      l.H(0, 0) = 1.0;
      l.H(1, 1) = 1.0;
      l.y(0) = c.local->position(0) - x(0);
      l.y(1) = c.local->position(1) - x(1);
      l.R = c.local->covariance;
      return AnyLinearized{l};
    }
  }
  return std::nullopt;
}

std::optional<double> mahalanobis2(const ConstantVelocityFilter& f,
                                   const AnyLinearized& l) noexcept {
  return std::visit([&f](const auto& v) { return f.mahalanobis2(v.y, v.H, v.R); }, l);
}

FilterStatus apply(ConstantVelocityFilter& f, const AnyLinearized& l) noexcept {
  return std::visit(
      [&f](const auto& v) {
        constexpr int M = std::remove_cvref_t<decltype(v.y)>::RowsAtCompileTime;
        return f.template update_with_innovation<M>(v.y, v.H, v.R);
      },
      l);
}

std::optional<LocalPosition> initial_position(const MeasurementContext& c) noexcept {
  const Measurement& m = *c.m;
  switch (m.kind) {
    case SensorKind::RangeBearing:
      return range_bearing_to_local({m.z[0], m.z[1]}, {m.sigma[0], m.sigma[1]}, c.pose);
    case SensorKind::GeoPosition:
      if (!c.local) {
        return std::nullopt;
      }
      return LocalPosition{c.local->position, c.local->covariance};
    case SensorKind::BearingOnly:
      return std::nullopt;
  }
  return std::nullopt;
}

}  // namespace sifted::estimation
