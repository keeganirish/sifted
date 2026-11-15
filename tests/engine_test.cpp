#include "sifted/core/engine.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "sifted/estimation/range_bearing.hpp"

namespace sifted {
namespace {

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr SensorId kRadar = 4;
constexpr SensorId kCamera = 5;
const SensorPose kPose{200.0, -100.0, 0.3};

SensorTable table() {
  SensorTable t;
  EXPECT_FALSE(t.add({kRadar, SensorKind::RangeBearing, kPose}));
  EXPECT_FALSE(t.add({kCamera, SensorKind::BearingOnly, {0.0, 0.0, 0.0}}));
  return t;
}

// Noise-free radar measurement of a point target at (x, y).
Measurement radar_at(double x, double y, std::int64_t t_ns, std::uint32_t seq) {
  estimation::StateVector s = estimation::StateVector::Zero();
  s(0) = x;
  s(1) = y;
  const estimation::RangeBearing z = estimation::predict_range_bearing(s, kPose);
  Measurement m;
  m.sensor_id = kRadar;
  m.kind = SensorKind::RangeBearing;
  m.seq = seq;
  m.time = Timestamp{t_ns};
  m.z = {z.range_m, z.bearing_rad, 0.0};
  m.sigma = {10.0, 0.0035, 0.0};
  return m;
}

}  // namespace

TEST(Engine, RejectsMeasurementsItCannotUse) {
  Engine e(table());
  Measurement bad = radar_at(1000, 1000, 0, 0);
  bad.z[1] = 4.0;  // bearing out of range
  EXPECT_EQ(e.on_measurement(bad), MeasurementOutcome::Invalid);

  Measurement unknown = radar_at(1000, 1000, 0, 0);
  unknown.sensor_id = 99;
  EXPECT_EQ(e.on_measurement(unknown), MeasurementOutcome::UnknownSensor);

  Measurement wrong_kind;  // bearing-only data claiming to come from the radar
  wrong_kind.sensor_id = kRadar;
  wrong_kind.kind = SensorKind::BearingOnly;
  wrong_kind.z = {0.1, 0.0, 0.0};
  wrong_kind.sigma = {0.01, 0.0, 0.0};
  EXPECT_EQ(e.on_measurement(wrong_kind), MeasurementOutcome::KindMismatch);

  wrong_kind.sensor_id = kCamera;
  EXPECT_EQ(e.on_measurement(wrong_kind), MeasurementOutcome::UnsupportedKind);

  EXPECT_FALSE(e.track().has_value());
  EXPECT_EQ(e.measurements_processed(), 4U);
  EXPECT_EQ(e.count(MeasurementOutcome::Invalid), 1U);
  EXPECT_EQ(e.count(MeasurementOutcome::UnsupportedKind), 1U);
}

TEST(Engine, InitializesFromFirstMeasurement) {
  Engine e(table());
  ASSERT_EQ(e.on_measurement(radar_at(3000.0, 4000.0, 5 * kSecond, 0)),
            MeasurementOutcome::Initialized);
  const auto t = e.track();
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->id, 1U);
  EXPECT_EQ(t->time, Timestamp{5 * kSecond});
  EXPECT_EQ(t->updates, 1U);
  EXPECT_EQ(t->last_sensor, kRadar);
  EXPECT_NEAR(t->state(0), 3000.0, 1e-6);
  EXPECT_NEAR(t->state(1), 4000.0, 1e-6);
  EXPECT_EQ(t->state(2), 0.0);
  EXPECT_DOUBLE_EQ(t->covariance(2, 2),
                   e.config().initial_velocity_sigma * e.config().initial_velocity_sigma);
}

TEST(Engine, TracksConstantVelocityTarget) {
  Engine e(table());
  for (std::uint32_t k = 0; k <= 60; ++k) {
    const double s = static_cast<double>(k);
    const auto o = e.on_measurement(
        radar_at(-4000.0 + 50.0 * s, 2000.0 - 20.0 * s, static_cast<std::int64_t>(k) * kSecond, k));
    ASSERT_EQ(o, k == 0 ? MeasurementOutcome::Initialized : MeasurementOutcome::Updated);
  }
  const auto t = e.track();
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->updates, 61U);
  EXPECT_NEAR(t->state(0), -1000.0, 2.0);
  EXPECT_NEAR(t->state(1), 800.0, 2.0);
  EXPECT_NEAR(t->state(2), 50.0, 0.5);
  EXPECT_NEAR(t->state(3), -20.0, 0.5);

  const auto ahead = e.track_at(Timestamp{70 * kSecond});
  ASSERT_TRUE(ahead.has_value());
  EXPECT_NEAR(ahead->state(0), -500.0, 10.0);
  EXPECT_GT(ahead->covariance(0, 0), t->covariance(0, 0));  // uncertainty grows
  EXPECT_EQ(e.track()->time, Timestamp{60 * kSecond});      // track_at is read-only
  EXPECT_FALSE(e.track_at(Timestamp{59 * kSecond}).has_value());
}

TEST(Engine, DropsStaleAndDuplicateMeasurementsWithoutTouchingTheTrack) {
  Engine e(table());
  ASSERT_EQ(e.on_measurement(radar_at(1000, 2000, 10 * kSecond, 0)),
            MeasurementOutcome::Initialized);
  ASSERT_EQ(e.on_measurement(radar_at(1010, 2000, 11 * kSecond, 1)), MeasurementOutcome::Updated);
  const auto before = e.track();

  EXPECT_EQ(e.on_measurement(radar_at(900, 2000, 9 * kSecond, 2)), MeasurementOutcome::OutOfOrder);
  EXPECT_EQ(e.on_measurement(radar_at(1010, 2000, 11 * kSecond, 1)), MeasurementOutcome::Duplicate);

  const auto after = e.track();
  EXPECT_TRUE(after->state == before->state);
  EXPECT_TRUE(after->covariance == before->covariance);
  EXPECT_EQ(after->updates, 2U);
  EXPECT_EQ(e.count(MeasurementOutcome::OutOfOrder), 1U);
  EXPECT_EQ(e.count(MeasurementOutcome::Duplicate), 1U);

  // A same-time measurement with a new seq is a legitimate second look (dt = 0).
  EXPECT_EQ(e.on_measurement(radar_at(1010, 2000, 11 * kSecond, 3)), MeasurementOutcome::Updated);
}

TEST(Engine, FilterRejectionLeavesStateUnchanged) {
  Engine e(table());
  // Zero range gives a singular initial covariance: refused, no track.
  Measurement at_sensor = radar_at(1000, 0, 0, 0);
  at_sensor.z[0] = 0.0;
  EXPECT_EQ(e.on_measurement(at_sensor), MeasurementOutcome::FilterRejected);
  EXPECT_FALSE(e.track().has_value());

  ASSERT_EQ(e.on_measurement(radar_at(1000, 0, kSecond, 1)), MeasurementOutcome::Initialized);
  const auto before = e.track();
  // A gap longer than the filter's maximum prediction step is refused.
  const auto far =
      static_cast<std::int64_t>(estimation::ConstantVelocityFilter::kMaxPredictDt) + 10;
  EXPECT_EQ(e.on_measurement(radar_at(1000, 0, far * kSecond, 2)),
            MeasurementOutcome::FilterRejected);
  EXPECT_TRUE(e.track()->state == before->state);
  EXPECT_EQ(e.track()->time, before->time);
}

TEST(Engine, ExtremeTimestampsDoNotOverflow) {
  Engine e(table());
  ASSERT_EQ(e.on_measurement(radar_at(1000, 0, INT64_MIN, 0)), MeasurementOutcome::Initialized);
  // ~2^64 ns later: a huge but well-defined gap, refused by the filter.
  EXPECT_EQ(e.on_measurement(radar_at(1000, 0, INT64_MAX, 1)), MeasurementOutcome::FilterRejected);
}

TEST(Engine, IdenticalInputGivesBitIdenticalOutput) {
  Engine a(table());
  Engine b(table());
  for (std::uint32_t k = 0; k < 30; ++k) {
    const double s = static_cast<double>(k);
    const Measurement m = radar_at(500.0 + 30.0 * s + 3.0 * std::sin(s), 800.0 - 10.0 * s,
                                   static_cast<std::int64_t>(k) * kSecond / 2, k);
    ASSERT_EQ(a.on_measurement(m), b.on_measurement(m));
  }
  EXPECT_TRUE(a.track()->state == b.track()->state);
  EXPECT_TRUE(a.track()->covariance == b.track()->covariance);
}

}  // namespace sifted
