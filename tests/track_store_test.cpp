// Track store and decision trace: "why does the system believe this track?"

#include "sifted/store/track_store.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "sifted/core/engine.hpp"
#include "sifted/estimation/range_bearing.hpp"

namespace sifted {
namespace {

constexpr std::int64_t kSecond = 1'000'000'000;
const SensorPose kPose{0.0, 0.0, 0.0};

Measurement radar_at(double x, double y, std::int64_t t_ns, std::uint32_t seq) {
  estimation::StateVector s = estimation::StateVector::Zero();
  s(0) = x;
  s(1) = y;
  const estimation::RangeBearing z = estimation::predict_range_bearing(s, kPose);
  Measurement m;
  m.sensor_id = 1;
  m.kind = SensorKind::RangeBearing;
  m.seq = seq;
  m.time = Timestamp{t_ns};
  m.z = {z.range_m, z.bearing_rad, 0.0};
  m.sigma = {10.0, 0.0035, 0.0};
  return m;
}

SensorTable table() {
  SensorTable t;
  EXPECT_FALSE(t.add({1, SensorKind::RangeBearing, kPose}));
  return t;
}

bool contains(const std::string& text, const std::string& what) {
  return text.find(what) != std::string::npos;
}

}  // namespace

TEST(Ring, KeepsTheNewestItemsInOrder) {
  store::Ring<int> r(3);
  for (int i = 1; i <= 5; ++i) {
    r.push(i);
  }
  ASSERT_EQ(r.size(), 3U);
  EXPECT_EQ(r.pushed(), 5U);
  EXPECT_EQ(r.at(0), 3);
  EXPECT_EQ(r.at(2), 5);
  EXPECT_EQ(r.back(), 5);
}

TEST(Trace, ExplainsATrackEndToEnd) {
  Engine e(table());
  std::uint32_t seq = 0;
  for (int k = 0; k < 5; ++k) {
    e.on_measurement(radar_at(4000.0 + 50.0 * k, 0.0, k * kSecond, seq++));
  }
  // A clutter point 130 m off: outside the gate (~75 m), inside the near-miss band.
  e.on_measurement(radar_at(4250.0, 130.0, 5 * kSecond, seq++));

  const auto text = e.explain(1);
  ASSERT_TRUE(text.has_value());
  EXPECT_TRUE(contains(*text, "track 1 (confirmed)")) << *text;
  EXPECT_TRUE(contains(*text, "born: t=0 initiated sensor 1 seq 0")) << *text;
  EXPECT_TRUE(contains(*text, "confirmed: t=2000000000 confirmed")) << *text;
  EXPECT_TRUE(contains(*text, "associated sensor 1 seq 4 d2=")) << *text;
  EXPECT_TRUE(contains(*text, "sensors: 1 (5 updates")) << *text;

  const store::TrackRecord* r = e.store().find(1);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(r->totals[static_cast<std::size_t>(store::TraceEventKind::Associated)], 4U);
  EXPECT_EQ(r->totals[static_cast<std::size_t>(store::TraceEventKind::Initiated)], 1U);

  // Every measurement's outcome is logged, in order.
  const auto& log = e.store().measurements();
  ASSERT_EQ(log.size(), 6U);
  EXPECT_EQ(log.at(0).outcome, MeasurementOutcome::Initialized);
  EXPECT_EQ(log.at(4).outcome, MeasurementOutcome::Updated);
  EXPECT_EQ(log.at(4).track, 1U);
}

TEST(Trace, NearMissesAndCompetitionAreRecorded) {
  Engine e(table());
  for (int k = 0; k < 4; ++k) {
    e.on_measurement(radar_at(4000.0, 0.0, k * kSecond, static_cast<std::uint32_t>(k)));
  }
  // 130 m off is a near miss (outside the ~75 m gate) for track 1 and starts track 2.
  const MeasurementResult r = e.on_measurement_result(radar_at(4000.0, 130.0, 4 * kSecond, 10));
  ASSERT_EQ(r.track, 2U);
  const store::TrackRecord* t1 = e.store().find(1);
  ASSERT_NE(t1, nullptr);
  EXPECT_EQ(t1->totals[static_cast<std::size_t>(store::TraceEventKind::NearMiss)], 1U);
  EXPECT_TRUE(contains(*e.explain(1), "near miss sensor 1 seq 10 d2="));
}

TEST(Trace, IsBoundedAndSurvivesTheDrop) {
  EngineConfig cfg;
  cfg.trace.events_per_track = 4;
  cfg.trace.measurement_log = 8;
  cfg.trace.dropped_tracks_kept = 1;
  Engine e(table(), cfg);
  for (int k = 0; k < 20; ++k) {
    e.on_measurement(radar_at(4000.0, 0.0, k * kSecond, static_cast<std::uint32_t>(k)));
  }
  const store::TrackRecord* r = e.store().find(1);
  ASSERT_NE(r, nullptr);
  EXPECT_EQ(r->recent.size(), 4U);
  EXPECT_GT(r->recent.pushed(), 4U);
  EXPECT_TRUE(r->initiated.has_value());  // birth is kept even after the window rolls
  EXPECT_EQ(e.store().measurements().size(), 8U);

  // Far detections 15 s later drop track 1 (stale), then track 2 times out.
  e.on_measurement(radar_at(-4000.0, 0.0, 35 * kSecond, 100));
  e.on_measurement(radar_at(4000.0, 4000.0, 40 * kSecond, 101));
  ASSERT_FALSE(e.track(1).has_value());
  EXPECT_EQ(e.store().dropped().size(), 1U);  // history is bounded too
  EXPECT_FALSE(e.explain(1).has_value());     // evicted by track 2's drop
  const auto text = e.explain(2);
  ASSERT_TRUE(text.has_value());
  EXPECT_TRUE(contains(*text, "(dropped)")) << *text;
  EXPECT_TRUE(contains(*text, "tentative timeout")) << *text;
}

}  // namespace sifted
