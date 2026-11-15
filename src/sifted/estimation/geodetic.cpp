#include "sifted/estimation/geodetic.hpp"

#include <cmath>

namespace sifted::estimation {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
// WGS-84
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);

// Rows of the ECEF -> ENU rotation at (lat, lon).
struct EnuBasis {
  Eigen::Vector3d east;
  Eigen::Vector3d north;
  Eigen::Vector3d up;
};

EnuBasis basis(double lat_rad, double lon_rad) noexcept {
  const double sl = std::sin(lat_rad);
  const double cl = std::cos(lat_rad);
  const double so = std::sin(lon_rad);
  const double co = std::cos(lon_rad);
  return {Eigen::Vector3d(-so, co, 0.0), Eigen::Vector3d(-sl * co, -sl * so, cl),
          Eigen::Vector3d(cl * co, cl * so, sl)};
}

Geodetic as_geodetic(const GeoOrigin& o) noexcept { return {o.lat_deg, o.lon_deg, o.alt_m}; }

}  // namespace

bool is_valid(const GeoOrigin& o) noexcept {
  return o.lat_deg >= -90.0 && o.lat_deg <= 90.0 && o.lon_deg >= -180.0 && o.lon_deg <= 180.0 &&
         o.alt_m >= -1000.0 && o.alt_m <= 100000.0;  // comparisons are false for NaN
}

Eigen::Vector3d geodetic_to_ecef(const Geodetic& g) noexcept {
  const double lat = g.lat_deg * kDegToRad;
  const double lon = g.lon_deg * kDegToRad;
  const double sl = std::sin(lat);
  const double cl = std::cos(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
  return Eigen::Vector3d((n + g.alt_m) * cl * std::cos(lon), (n + g.alt_m) * cl * std::sin(lon),
                         (n * (1.0 - kE2) + g.alt_m) * sl);
}

Geodetic ecef_to_geodetic(const Eigen::Vector3d& ecef) noexcept {
  // Fixed-count fixed-point iteration on latitude (converges to well below a
  // millimetre for terrestrial points in a few steps). A fixed count keeps the
  // result a deterministic function of the input.
  const double x = ecef(0);
  const double y = ecef(1);
  const double z = ecef(2);
  const double p = std::sqrt(x * x + y * y);
  const double lon = std::atan2(y, x);
  double lat = std::atan2(z, p * (1.0 - kE2));
  double h = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double sl = std::sin(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
    const double cl = std::cos(lat);
    h = std::abs(cl) > 1e-12 ? p / cl - n : std::abs(z) - n * (1.0 - kE2);
    lat = std::atan2(z, p * (1.0 - kE2 * n / (n + h)));
  }
  return {lat * kRadToDeg, lon * kRadToDeg, h};
}

Eigen::Vector3d geodetic_to_enu(const GeoOrigin& origin, const Geodetic& g) noexcept {
  const Eigen::Vector3d d = geodetic_to_ecef(g) - geodetic_to_ecef(as_geodetic(origin));
  const EnuBasis b = basis(origin.lat_deg * kDegToRad, origin.lon_deg * kDegToRad);
  return Eigen::Vector3d(b.east.dot(d), b.north.dot(d), b.up.dot(d));
}

Geodetic enu_to_geodetic(const GeoOrigin& origin, const Eigen::Vector3d& enu) noexcept {
  const EnuBasis b = basis(origin.lat_deg * kDegToRad, origin.lon_deg * kDegToRad);
  const Eigen::Vector3d ecef =
      geodetic_to_ecef(as_geodetic(origin)) + b.east * enu(0) + b.north * enu(1) + b.up * enu(2);
  return ecef_to_geodetic(ecef);
}

std::optional<LocalPositionMeasurement> geodetic_measurement_to_local(
    const GeoOrigin& origin, const Geodetic& z, const Geodetic& sigma) noexcept {
  const Eigen::Vector3d enu = geodetic_to_enu(origin, z);
  if (!enu.allFinite() ||
      !(std::sqrt(enu(0) * enu(0) + enu(1) * enu(1)) <= kMaxTangentPlaneRangeM)) {
    return std::nullopt;
  }

  // d(ECEF)/d(lat), d(ECEF)/d(lon) at the measured point, in metres per radian.
  const double lat = z.lat_deg * kDegToRad;
  const double lon = z.lon_deg * kDegToRad;
  const double sl = std::sin(lat);
  const double cl = std::cos(lat);
  const double so = std::sin(lon);
  const double co = std::cos(lon);
  const double w = std::sqrt(1.0 - kE2 * sl * sl);
  const double n = kA / w;                          // prime vertical radius
  const double m = kA * (1.0 - kE2) / (w * w * w);  // meridian radius
  const Eigen::Vector3d d_lat(-(m + z.alt_m) * sl * co, -(m + z.alt_m) * sl * so,
                              (m + z.alt_m) * cl);
  const Eigen::Vector3d d_lon(-(n + z.alt_m) * cl * so, (n + z.alt_m) * cl * co, 0.0);

  const EnuBasis b = basis(origin.lat_deg * kDegToRad, origin.lon_deg * kDegToRad);
  Eigen::Matrix2d J;
  J(0, 0) = b.east.dot(d_lat);
  J(0, 1) = b.east.dot(d_lon);
  J(1, 0) = b.north.dot(d_lat);
  J(1, 1) = b.north.dot(d_lon);
  Eigen::Matrix2d Rz = Eigen::Matrix2d::Zero();
  const double s_lat = sigma.lat_deg * kDegToRad;
  const double s_lon = sigma.lon_deg * kDegToRad;
  Rz(0, 0) = s_lat * s_lat;
  Rz(1, 1) = s_lon * s_lon;

  LocalPositionMeasurement out;
  out.position = Eigen::Vector2d(enu(0), enu(1));
  out.covariance = J * Rz * J.transpose();
  out.covariance = 0.5 * (out.covariance + out.covariance.transpose());
  if (!out.covariance.allFinite()) {
    return std::nullopt;
  }
  return out;
}

}  // namespace sifted::estimation
