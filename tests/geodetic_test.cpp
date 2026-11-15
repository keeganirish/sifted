// Geodetic <-> tangent-plane conversion (ADS-B ingest, map display).

#include "sifted/estimation/geodetic.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace sifted::estimation {

TEST(Geodetic, EcefOfKnownPoints) {
  const Eigen::Vector3d equator = geodetic_to_ecef({0.0, 0.0, 0.0});
  EXPECT_NEAR(equator(0), 6378137.0, 1e-6);
  EXPECT_NEAR(equator(1), 0.0, 1e-6);
  const Eigen::Vector3d pole = geodetic_to_ecef({90.0, 0.0, 0.0});
  EXPECT_NEAR(pole(2), 6356752.314245, 1e-5);  // WGS-84 semi-minor axis
}

TEST(Geodetic, RoundTripsThroughEcefAndEnu) {
  const GeoOrigin o{51.4779, -0.0015, 46.0};
  for (const Geodetic g : {Geodetic{51.6, 0.3, 11000.0}, Geodetic{50.9, -1.4, 300.0},
                           Geodetic{51.4779, -0.0015, 46.0}}) {
    const Geodetic back = ecef_to_geodetic(geodetic_to_ecef(g));
    EXPECT_NEAR(back.lat_deg, g.lat_deg, 1e-9);
    EXPECT_NEAR(back.lon_deg, g.lon_deg, 1e-9);
    EXPECT_NEAR(back.alt_m, g.alt_m, 1e-4);
    const Geodetic via_enu = enu_to_geodetic(o, geodetic_to_enu(o, g));
    EXPECT_NEAR(via_enu.lat_deg, g.lat_deg, 1e-9);
    EXPECT_NEAR(via_enu.lon_deg, g.lon_deg, 1e-9);
    EXPECT_NEAR(via_enu.alt_m, g.alt_m, 1e-4);
  }
}

TEST(Geodetic, EnuAxesPointEastAndNorth) {
  const GeoOrigin o{40.0, -100.0, 0.0};
  const Eigen::Vector3d north = geodetic_to_enu(o, {40.01, -100.0, 0.0});
  EXPECT_NEAR(north(0), 0.0, 1e-6);
  EXPECT_NEAR(north(1), 1110.5, 1.0);  // ~111 km per degree of latitude at 40N
  const Eigen::Vector3d east = geodetic_to_enu(o, {40.0, -99.99, 0.0});
  EXPECT_NEAR(east(0), 853.0, 1.0);  // shrinks with cos(latitude)
  EXPECT_NEAR(east(1), 0.0, 0.1);
}

TEST(Geodetic, MeasurementCovarianceScalesDegreesToMetres) {
  const GeoOrigin o{0.0, 0.0, 0.0};
  const double one_m_lat = 1.0 / 110574.0;  // degrees of latitude per metre at the equator
  const auto m =
      geodetic_measurement_to_local(o, {0.0, 0.0, 0.0}, {10 * one_m_lat, 10 * one_m_lat, 5.0});
  ASSERT_TRUE(m.has_value());
  EXPECT_NEAR(std::sqrt(m->covariance(1, 1)), 10.0, 0.01);  // north
  EXPECT_GT(m->covariance(0, 0), 0.0);
  EXPECT_NEAR(m->covariance(0, 1), 0.0, 1e-9);
}

TEST(Geodetic, RefusesPointsBeyondTheTangentPlaneRange) {
  const GeoOrigin o{0.0, 0.0, 0.0};
  EXPECT_TRUE(geodetic_measurement_to_local(o, {8.0, 0.0, 0.0}, {1e-4, 1e-4, 1.0}).has_value());
  EXPECT_FALSE(geodetic_measurement_to_local(o, {10.0, 0.0, 0.0}, {1e-4, 1e-4, 1.0}).has_value());
  EXPECT_FALSE(is_valid(GeoOrigin{91.0, 0.0, 0.0}));
  EXPECT_TRUE(is_valid(GeoOrigin{-33.9, 151.2, 10.0}));
}

}  // namespace sifted::estimation
