#include "sifted/core/measurement.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "sifted/core/engine.hpp"

namespace sifted {
namespace {

Measurement bearing_only() {
  Measurement m;
  m.kind = SensorKind::BearingOnly;
  m.z = {1.0, 0.0, 0.0};
  m.sigma = {0.01, 0.0, 0.0};
  return m;
}

Measurement range_bearing() {
  Measurement m;
  m.kind = SensorKind::RangeBearing;
  m.z = {5000.0, -1.0, 0.0};
  m.sigma = {10.0, 0.01, 0.0};
  return m;
}

Measurement geo() {
  Measurement m;
  m.kind = SensorKind::GeoPosition;
  m.z = {41.9, -87.6, 10000.0};
  m.sigma = {1e-4, 1e-4, 30.0};
  return m;
}

}  // namespace

TEST(Measurement, TypicalValuesAreValid) {
  EXPECT_TRUE(is_valid(bearing_only()));
  EXPECT_TRUE(is_valid(range_bearing()));
  EXPECT_TRUE(is_valid(geo()));
}

TEST(Measurement, RejectsNonFinite) {
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  constexpr double inf = std::numeric_limits<double>::infinity();
  for (double bad : {nan, inf, -inf}) {
    auto m = range_bearing();
    m.z[0] = bad;
    EXPECT_FALSE(is_valid(m)) << bad;
    m = range_bearing();
    m.sigma[1] = bad;
    EXPECT_FALSE(is_valid(m)) << bad;
  }
}

TEST(Measurement, RejectsOutOfRange) {
  auto m = range_bearing();
  m.z[0] = -1.0;
  EXPECT_FALSE(is_valid(m));
  m = bearing_only();
  m.z[0] = 3.2;  // > pi
  EXPECT_FALSE(is_valid(m));
  m = geo();
  m.z[0] = 90.5;
  EXPECT_FALSE(is_valid(m));
}

TEST(Measurement, RejectsBadSigma) {
  auto m = bearing_only();
  m.sigma[0] = 0.0;
  EXPECT_FALSE(is_valid(m));
  m.sigma[0] = -0.1;
  EXPECT_FALSE(is_valid(m));
  m.sigma[0] = 1e7;
  EXPECT_FALSE(is_valid(m));
}

TEST(Measurement, RejectsNonCanonicalUnusedComponents) {
  auto m = bearing_only();
  m.z[2] = 1.0;
  EXPECT_FALSE(is_valid(m));
  m = bearing_only();
  m.sigma[1] = 0.1;
  EXPECT_FALSE(is_valid(m));
  m = bearing_only();
  m.z[1] = -0.0;  // numerically zero but not the canonical bit pattern
  EXPECT_FALSE(is_valid(m));
}

TEST(Measurement, RejectsUnknownKind) {
  auto m = bearing_only();
  m.kind = static_cast<SensorKind>(0);
  EXPECT_FALSE(is_valid(m));
  m.kind = static_cast<SensorKind>(200);
  EXPECT_FALSE(is_valid(m));
}

TEST(Engine, CountsProcessedMeasurements) {
  Engine e;
  EXPECT_EQ(e.measurements_processed(), 0U);
  e.on_measurement(bearing_only());
  e.on_measurement(bearing_only());
  EXPECT_EQ(e.measurements_processed(), 2U);
}

}  // namespace sifted
