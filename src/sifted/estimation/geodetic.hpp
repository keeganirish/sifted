#pragma once

#include <Eigen/Core>
#include <optional>

// Geodetic (WGS-84) <-> local tangent plane conversion.
//
// The tracking frame (x east, y north, metres) is the East-North-Up tangent
// plane at a GeoOrigin that is fixed for the whole run and recorded in the
// event log header, so a replay projects every geodetic measurement exactly as
// the live run did (ADR 0007). The tracker is 2-D: "up" is dropped after the
// projection. The flat-plane approximation degrades with distance from the
// origin, so points more than kMaxTangentPlaneRangeM away are refused.
namespace sifted::estimation {

struct GeoOrigin {
  double lat_deg = 0.0;  // [-90, 90]
  double lon_deg = 0.0;  // [-180, 180]
  double alt_m = 0.0;

  friend bool operator==(const GeoOrigin&, const GeoOrigin&) = default;
};

struct Geodetic {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double alt_m = 0.0;
};

// Horizontal distance from the origin beyond which a projected point is
// refused: at 1000 km the tangent plane is ~78 km above the ellipsoid, and
// ranges and bearings in the flat frame are visibly distorted.
inline constexpr double kMaxTangentPlaneRangeM = 1.0e6;

// Finite, latitude in [-90, 90], longitude in [-180, 180], altitude in a sane band.
bool is_valid(const GeoOrigin& o) noexcept;

Eigen::Vector3d geodetic_to_ecef(const Geodetic& g) noexcept;
Geodetic ecef_to_geodetic(const Eigen::Vector3d& ecef) noexcept;

// ENU coordinates of `g` in the tangent plane at `origin`.
Eigen::Vector3d geodetic_to_enu(const GeoOrigin& origin, const Geodetic& g) noexcept;
// Inverse: the geodetic point at ENU offset `enu` from `origin`.
Geodetic enu_to_geodetic(const GeoOrigin& origin, const Eigen::Vector3d& enu) noexcept;

// A geodetic position measurement projected into the tracking frame.
struct LocalPositionMeasurement {
  Eigen::Vector2d position;    // [east, north] in metres
  Eigen::Matrix2d covariance;  // first-order propagation of the lat/lon noise
};

// Projects (lat, lon, alt) with 1-sigma noise `sigma` (degrees, degrees, metres)
// into the tracking frame. The covariance is J diag(sigma_lat^2, sigma_lon^2) J^T
// with J = d(east, north)/d(lat, lon) evaluated at the measured point, so it is
// correct away from the origin too. Altitude noise is ignored (2-D tracker).
// nullopt if the point is further than kMaxTangentPlaneRangeM from the origin.
std::optional<LocalPositionMeasurement> geodetic_measurement_to_local(
    const GeoOrigin& origin, const Geodetic& z, const Geodetic& sigma) noexcept;

}  // namespace sifted::estimation
