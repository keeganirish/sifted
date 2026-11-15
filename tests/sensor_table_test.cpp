#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "sifted/core/sensor.hpp"

namespace sifted {
namespace {

constexpr double kPi = 3.14159265358979323846;

SensorConfig radar(SensorId id, double x = 0.0, double y = 0.0, double yaw = 0.0) {
  return {id, SensorKind::RangeBearing, {x, y, yaw}};
}

SensorTableError error_of(std::string_view text) {
  const auto r = parse_sensor_table(text);
  EXPECT_FALSE(r.ok()) << text;
  return r.ok() ? SensorTableError::Malformed : r.error();
}

}  // namespace

TEST(SensorTable, KeepsEntriesSortedById) {
  SensorTable t;
  EXPECT_FALSE(t.add(radar(9)));
  EXPECT_FALSE(t.add(radar(2)));
  EXPECT_FALSE(t.add(radar(5)));
  ASSERT_EQ(t.size(), 3U);
  EXPECT_EQ(t.sensors()[0].id, 2);
  EXPECT_EQ(t.sensors()[2].id, 9);
  EXPECT_EQ(t.index_of(5), 1U);
  EXPECT_EQ(t.find(9)->id, 9);
  EXPECT_EQ(t.find(4), nullptr);
  EXPECT_FALSE(t.index_of(4).has_value());
}

TEST(SensorTable, RejectsDuplicatesAndInvalidPoses) {
  SensorTable t;
  ASSERT_FALSE(t.add(radar(1)));
  EXPECT_EQ(t.add(radar(1, 5.0)), SensorTableError::DuplicateId);
  EXPECT_EQ(t.add(radar(2, std::numeric_limits<double>::quiet_NaN())),
            SensorTableError::InvalidSensor);
  EXPECT_EQ(t.add(radar(3, 0.0, 2e7)), SensorTableError::InvalidSensor);
  EXPECT_EQ(t.add(radar(4, 0.0, 0.0, 3.5)), SensorTableError::InvalidSensor);
  SensorConfig bad = radar(5);
  bad.kind = static_cast<SensorKind>(0);
  EXPECT_EQ(t.add(bad), SensorTableError::InvalidSensor);
  EXPECT_EQ(t.size(), 1U);  // failed adds leave the table unchanged
}

TEST(SensorTable, ParsesTextForm) {
  const auto r = parse_sensor_table(
      "# test table\n"
      "sifted-sensors 1\n"
      "\n"
      "sensor 2 range_bearing 1000.0 -500.0 0.5\r\n"
      "  sensor\t1 bearing_only -1e3 2.5 -3.141592653589793\n");
  ASSERT_TRUE(r.ok()) << to_string(r.error());
  const SensorTable& t = r.value();
  ASSERT_EQ(t.size(), 2U);
  EXPECT_EQ(*t.find(2), radar(2, 1000.0, -500.0, 0.5));
  EXPECT_EQ(t.find(1)->kind, SensorKind::BearingOnly);
  EXPECT_EQ(t.find(1)->pose.yaw_rad, -kPi);
}

TEST(SensorTable, FormatRoundTripsBitExactly) {
  SensorTable t;
  ASSERT_FALSE(t.add(radar(7, 0.1, -2500.0, -kPi)));
  ASSERT_FALSE(t.add(radar(3, 1e-17, 1.0 / 3.0, 0.3)));
  const std::string text = format_sensor_table(t);
  EXPECT_EQ(text.rfind("sifted-sensors 1\nsensor 3 range_bearing", 0), 0U) << text;
  const auto back = parse_sensor_table(text);
  ASSERT_TRUE(back.ok());
  EXPECT_EQ(back.value(), t);
  EXPECT_EQ(format_sensor_table(back.value()), text);
}

TEST(SensorTable, RejectsMalformedText) {
  EXPECT_EQ(error_of(""), SensorTableError::MissingHeader);
  EXPECT_EQ(error_of("sensor 1 range_bearing 0 0 0\n"), SensorTableError::MissingHeader);
  EXPECT_EQ(error_of("sifted-sensors 2\n"), SensorTableError::UnsupportedVersion);
  EXPECT_EQ(error_of("sifted-sensors 1\nsensor 1 range_bearing 0 0\n"),
            SensorTableError::Malformed);
  EXPECT_EQ(error_of("sifted-sensors 1\nsensor 1 range_bearing 0 0 0 0\n"),
            SensorTableError::Malformed);
  EXPECT_EQ(error_of("sifted-sensors 1\nsensor 1 lidar 0 0 0\n"), SensorTableError::Malformed);
  EXPECT_EQ(error_of("sifted-sensors 1\nsensor 70000 range_bearing 0 0 0\n"),
            SensorTableError::Malformed);
  EXPECT_EQ(error_of("sifted-sensors 1\nsensor 1 range_bearing 0x10 0 0\n"),
            SensorTableError::Malformed);
  EXPECT_EQ(error_of("sifted-sensors 1\nsensor 1 range_bearing 1.0m 0 0\n"),
            SensorTableError::Malformed);
  EXPECT_EQ(error_of("sifted-sensors 1\nsensor 1 range_bearing nan 0 0\n"),
            SensorTableError::InvalidSensor);
  EXPECT_EQ(
      error_of("sifted-sensors 1\nsensor 1 range_bearing 0 0 0\nsensor 1 bearing_only 0 0 0\n"),
      SensorTableError::DuplicateId);
  EXPECT_EQ(error_of(std::string(kMaxSensorTableTextSize + 1, '#')), SensorTableError::TooLarge);
}

}  // namespace sifted
