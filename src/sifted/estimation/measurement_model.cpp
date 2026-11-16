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

GateScreen make_screen(const MeasurementContext& c) noexcept {
  GateScreen s;
  const Measurement& m = *c.m;
  if (m.kind == SensorKind::RangeBearing) {
    const double a = c.pose.yaw_rad + m.z[1];
    s.kind = GateScreen::Kind::RangeBearing;
    s.sensor = {c.pose.x_m, c.pose.y_m};
    s.range = m.z[0];
    s.point = s.sensor + s.range * Eigen::Vector2d(std::cos(a), std::sin(a));
    s.var_range = m.sigma[0] * m.sigma[0];
    s.var_bearing = m.sigma[1] * m.sigma[1];
  } else if (m.kind == SensorKind::GeoPosition && c.local) {
    const Eigen::Matrix2d& R = c.local->covariance;
    const double half_trace = 0.5 * (R(0, 0) + R(1, 1));
    const double disc =
        std::sqrt(0.25 * (R(0, 0) - R(1, 1)) * (R(0, 0) - R(1, 1)) + R(0, 1) * R(0, 1));
    s.kind = GateScreen::Kind::Position;
    s.point = c.local->position;
    s.var_range = half_trace + disc;
  }
  if (!s.point.allFinite() || !std::isfinite(s.var_range) || !std::isfinite(s.var_bearing)) {
    s.kind = GateScreen::Kind::None;  // never screen what cannot be bounded
  }
  return s;
}

bool may_gate(const GateScreen& s, const Eigen::Vector2d& position, double lambda,
              double limit) noexcept {
  constexpr double kSafety = 1.0 + 1e-6;
  if (s.kind == GateScreen::Kind::None || !(lambda >= 0.0) || !std::isfinite(lambda) ||
      !position.allFinite()) {
    return true;
  }
  const double d2 = (s.point - position).squaredNorm();
  double bound = 0.0;
  if (s.kind == GateScreen::Kind::Position) {
    bound = limit * (lambda + s.var_range);
  } else {
    const double r = (position - s.sensor).norm();
    if (!(r > 0.0)) {
      return true;
    }
    bound = limit * (lambda * (1.0 + s.range / r) + s.var_range + s.range * r * s.var_bearing);
  }
  return !(d2 > kSafety * bound) || !std::isfinite(bound);
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
