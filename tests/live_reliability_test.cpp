// Live reliability (ADR 0012): a forged timestamp cannot take over the timeline,
// silence is reported honestly and deterministically, and a crashed recording
// can be recovered. Each mechanism also has to replay bit for bit.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "scenario_support.hpp"
#include "sifted/net/durable_file.hpp"
#include "sifted/net/ingest.hpp"
#include "sifted/net/websocket.hpp"
#include "sifted/replay/event_log.hpp"
#include "sifted/replay/replayer.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::net {
namespace {

constexpr std::int64_t kS = 1'000'000'000;
constexpr std::int64_t kForged = 4'000'000'000'000'000'000;

SensorTable table() {
  SensorTable t;
  EXPECT_FALSE(t.add({1, SensorKind::RangeBearing, {0.0, 0.0, 0.0}}));
  EXPECT_FALSE(t.add({2, SensorKind::RangeBearing, {100.0, 0.0, 0.0}}));
  return t;
}

std::vector<std::uint8_t> frame(SensorId sensor, std::uint32_t seq, std::int64_t t,
                                double range = 4000.0) {
  Measurement m;
  m.sensor_id = sensor;
  m.kind = SensorKind::RangeBearing;
  m.seq = seq;
  m.time = Timestamp{t};
  m.z = {range, 0.5, 0.0};
  m.sigma = {10.0, 0.0035, 0.0};
  return wire::encode(m);
}

PipelineConfig guarded() {
  PipelineConfig c;
  c.max_time_jump_s = 2.5;
  c.timeline_confirm_frames = 3;
  c.timeline_confirm_s = 2.5;
  return c;
}

PipelineConfig watched() {
  PipelineConfig c = guarded();
  c.feed_stale_factor = 3.0;
  c.feed_stale_min_s = 2.5;
  return c;
}

// A Pipeline whose whole input is also written to an event log, so any test can
// finish by proving that the replay reproduces what it saw.
class Harness {
 public:
  explicit Harness(PipelineConfig c, SensorTable sensors = table())
      : header_{std::move(sensors), c},
        writer_(log_, header_),
        p_(
            header_.sensors, header_.config, [this](std::string_view l) { out_.append(l); },
            [this](const std::string& j) { published_.push_back(j); }) {}

  void send(std::int64_t recv, const std::vector<std::uint8_t>& bytes) {
    last_recv_ = recv;
    ASSERT_TRUE(writer_.write_frame(Timestamp{recv}, bytes));
    p_.on_frame(Timestamp{recv}, bytes);
  }
  void tick(std::int64_t recv) {
    last_recv_ = recv;
    ASSERT_TRUE(writer_.write_tick(Timestamp{recv}));
    p_.on_tick(Timestamp{recv});
  }
  // One scan of two well separated targets, at measurement time t.
  void scan(std::int64_t recv, std::int64_t t, SensorId sensor = 1) {
    // Ranges repeat every 100 s, so they stay valid on any epoch.
    const double phase = static_cast<double>(((t % (100 * kS)) + 100 * kS) % (100 * kS)) / kS;
    send(recv, frame(sensor, seq_++, t, 4000.0 + 20.0 * phase));
    send(recv, frame(sensor, seq_++, t, 9000.0 - 20.0 * phase));
    tick(recv);
  }
  // Scans once a second, measurement time = offset + receive time.
  void scans(std::int64_t from_s, std::int64_t to_s, std::int64_t offset = 0) {
    for (std::int64_t s = from_s; s < to_s; ++s) {
      scan(s * kS, s * kS + offset);
    }
  }
  void finish(std::int64_t recv) { p_.finish(Timestamp{recv}); }

  // The replay of everything sent so far must equal the live output.
  void expect_replays_identically() {
    p_.finish(Timestamp{last_recv_});
    std::stringstream in(log_.str());
    auto reader = replay::EventLogReader::open(in);
    ASSERT_TRUE(reader.ok());
    std::string again;
    Pipeline q(reader.value().header().sensors, reader.value().header().config,
               [&](std::string_view l) { again.append(l); });
    ASSERT_TRUE(replay::replay(reader.value(), q).ok());
    EXPECT_EQ(out_, again);
  }

  Pipeline& p() { return p_; }
  const std::string& out() const { return out_; }
  const std::vector<std::string>& published() const { return published_; }
  std::string log() const { return log_.str(); }
  std::size_t confirmed() const {
    std::size_t n = 0;
    for (const TrackEstimate& e : p_.engine().tracks()) {
      n += e.status == association::TrackStatus::Confirmed ? 1U : 0U;
    }
    return n;
  }

 private:
  replay::RunHeader header_;
  std::stringstream log_;
  replay::EventLogWriter writer_;
  std::string out_;
  std::vector<std::string> published_;
  Pipeline p_;
  std::uint32_t seq_ = 0;
  std::int64_t last_recv_ = 0;
};

// ---- defect 1: a forged future timestamp -------------------------------------

TEST(TimelineGuard, OneForgedFrameIsRejectedAndChangesNothing) {
  Harness r(guarded());
  r.scans(0, 12);
  ASSERT_EQ(r.confirmed(), 2U);
  const auto clock = r.p().engine().time();
  r.send(12 * kS, frame(1, 777, kForged));
  EXPECT_EQ(r.p().engine().time(), clock);
  EXPECT_EQ(r.p().timeline_held(), 1U);
  r.scans(13, 43);
  EXPECT_EQ(r.p().timeline_rejected(), 1U);
  EXPECT_EQ(r.p().timeline_held(), 0U);
  EXPECT_EQ(r.p().engine().count(MeasurementOutcome::OutOfOrder), 0U);
  EXPECT_EQ(r.confirmed(), 2U);
  EXPECT_EQ(*r.p().engine().time(), Timestamp{42 * kS});
  EXPECT_EQ(r.out().find(std::to_string(kForged) + " initialized"), std::string::npos);
  r.expect_replays_identically();
}

TEST(TimelineGuard, DisabledByDefaultSoOlderRecordingsKeepTheirBehaviour) {
  Harness r{PipelineConfig{}};
  r.scans(0, 5);
  r.send(5 * kS, frame(1, 777, kForged));
  r.tick(5 * kS);
  EXPECT_EQ(*r.p().engine().time(), Timestamp{kForged});  // the defect this guard exists for
  EXPECT_EQ(r.p().timeline_rejected(), 0U);
}

TEST(TimelineGuard, AForgedFirstFrameDoesNotCaptureTheRun) {
  Harness r(guarded());
  r.send(0, frame(1, 1, kForged));
  EXPECT_FALSE(r.p().engine().time().has_value());
  r.scans(1, 20);
  EXPECT_EQ(r.p().timeline_rejected(), 1U);
  EXPECT_EQ(*r.p().engine().time(), Timestamp{19 * kS});
  EXPECT_EQ(r.confirmed(), 2U);
  r.expect_replays_identically();
}

TEST(TimelineGuard, ALoneForgedFirstFrameIsCorrectedWhenRealDataArrives) {
  // Nothing contradicts it for the confirmation time, so it is believed...
  Harness r(guarded());
  r.send(0, frame(1, 1, kForged));
  r.tick(3 * kS);
  EXPECT_EQ(*r.p().engine().time(), Timestamp{kForged});
  // ...until the genuine timeline has been heard, consistently, for that long.
  r.scans(4, 20);
  EXPECT_EQ(r.p().timeline_restarts(), 1U);
  EXPECT_EQ(*r.p().engine().time(), Timestamp{19 * kS});
  EXPECT_EQ(r.confirmed(), 2U);
  r.expect_replays_identically();
}

TEST(TimelineGuard, DuplicatesOfAForgedFrameAreNotCorroboration) {
  Harness r(guarded());
  r.scans(0, 8);
  for (int i = 0; i < 20; ++i) {
    r.send((8 + i) * kS, frame(1, 777, kForged));  // total silence otherwise
    r.tick((8 + i) * kS);
  }
  EXPECT_EQ(r.p().timeline_switches(), 0U);
  EXPECT_EQ(*r.p().engine().time(), Timestamp{7 * kS});
  EXPECT_EQ(r.p().timeline_rejected(), 19U);  // the first copy is still in quarantine
  r.scans(30, 40, -23 * kS + 0);              // the genuine sensors resume
  r.expect_replays_identically();
}

TEST(TimelineGuard, ARejectedFrameDoesNotSuppressTheGenuineFrameWithTheSameSeq) {
  Harness r(guarded());
  r.scans(0, 4);
  // The forger guesses the next sequence number; then the real frame arrives.
  r.send(4 * kS, frame(1, 8, kForged));
  r.send(4 * kS, frame(1, 8, 4 * kS));
  r.send(4 * kS, frame(1, 9, 4 * kS, 8920.0));
  r.tick(4 * kS);
  EXPECT_EQ(r.p().duplicates(), 0U);
  EXPECT_EQ(r.p().timeline_rejected(), 1U);
  EXPECT_EQ(*r.p().engine().time(), Timestamp{4 * kS});
  EXPECT_EQ(r.p().engine().count(MeasurementOutcome::Updated) +
                r.p().engine().count(MeasurementOutcome::Initialized),
            10U);
}

TEST(TimelineGuard, ARejectedFrameDoesNotSplitAScanOrEnterTheReorderStage) {
  PipelineConfig c = guarded();
  c.reorder_window_s = 0.5;
  Harness r(c);
  r.scans(0, 4);
  r.tick(4 * kS + kS / 2 + 1);
  const std::uint64_t scans = r.p().scans();
  r.send(5 * kS, frame(1, 100, 5 * kS, 4100.0));
  r.send(5 * kS, frame(1, 777, kForged));   // in the middle of the scan
  r.send(5 * kS, frame(99, 778, kForged));  // from a sensor nobody knows
  r.send(5 * kS, frame(1, 101, 5 * kS, 8900.0));
  EXPECT_EQ(r.p().held(), 2U);  // only the genuine frames are held
  r.tick(5 * kS + kS / 2 + 1);
  EXPECT_EQ(r.p().scans(), scans + 1);  // one joint scan
  EXPECT_EQ(r.p().reorder_late(), 0U);
  // The reorder stage still works: the watermark did not jump to the forged time.
  r.send(6 * kS, frame(1, 102, 6 * kS, 4120.0));
  EXPECT_EQ(r.p().reorder_late(), 0U);
  EXPECT_EQ(r.p().held(), 1U);
}

TEST(TimelineGuard, ExtremeTimestampsCannotOverflowIt) {
  for (const std::int64_t first : {std::numeric_limits<std::int64_t>::min(), std::int64_t{-1},
                                   std::int64_t{0}, std::numeric_limits<std::int64_t>::max()}) {
    Harness r(guarded());
    r.scans(0, 3, first > 0 ? -3 * kS : 0);
    for (const std::int64_t t :
         {std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::max(), first,
          std::int64_t{-1}}) {
      r.send(3 * kS, frame(1, 500, t));
      r.send(3 * kS, frame(2, 501, t));
    }
    r.tick(9 * kS);
    r.expect_replays_identically();
  }
}

TEST(TimelineGuard, AGenuineEpochChangeIsAdoptedWithinTheDocumentedBound) {
  for (const std::int64_t epoch : {std::int64_t{1'700'000'000} * kS, -100 * kS}) {
    Harness r(guarded());
    r.scans(0, 8);
    const std::uint64_t started = r.p().engine().tracks_started();
    // The sensor restarts on a new epoch. Bound: the confirmation time.
    r.scan(8 * kS, 8 * kS + epoch);
    r.scan(9 * kS, 9 * kS + epoch);
    EXPECT_EQ(*r.p().engine().time(), Timestamp{7 * kS}) << "too soon";
    r.scan(10 * kS + kS / 2, 10 * kS + kS / 2 + epoch);
    EXPECT_EQ(r.p().timeline_switches(), 1U);
    EXPECT_EQ(r.p().timeline_rejected(), 0U) << "no genuine frame may be lost";
    EXPECT_EQ(*r.p().engine().time(), Timestamp{10 * kS + kS / 2 + epoch});
    r.scans(11, 20, epoch);
    EXPECT_EQ(r.confirmed(), 2U);
    EXPECT_GT(r.p().engine().tracks_started(), started);
    EXPECT_EQ(r.p().timeline_restarts(), epoch < 0 ? 1U : 0U);
    EXPECT_EQ(r.p().engine().count(MeasurementOutcome::OutOfOrder), 0U);
    r.expect_replays_identically();
  }
}

TEST(TimelineGuard, OneSensorOnAnotherEpochCannotDisplaceAHealthyOne) {
  Harness r(guarded());
  for (std::int64_t s = 0; s < 30; ++s) {
    r.scan(s * kS, s * kS, 1);
    r.scan(s * kS + kS / 2, s * kS + 1'000 * kS, 2);  // sensor 2 restarted elsewhere
  }
  EXPECT_EQ(r.p().timeline_switches(), 0U);
  EXPECT_EQ(*r.p().engine().time(), Timestamp{29 * kS});
  EXPECT_EQ(r.confirmed(), 2U);
  EXPECT_GE(r.p().timeline_rejected(), 50U);
  r.expect_replays_identically();
}

TEST(TimelineGuard, LegitimateTimingsNeverTouchIt) {
  struct Case {
    const char* what;
    std::int64_t start;
    std::int64_t recv_per_scan;  // receive time between scans (0: no pacing)
    std::int64_t time_per_scan;
  };
  const std::vector<Case> cases = {
      {"simulated epoch near 0", 0, kS, kS},
      {"wall-clock epoch", std::int64_t{1'700'000'000} * kS, kS, kS},
      {"--speed 10", 0, kS / 10, kS},
      {"--speed 0.25", 0, 4 * kS, kS},
      {"--speed 0 (no pacing)", 0, 0, kS},
      {"ADS-B polled every 22 s", 0, 22 * kS, 22 * kS},
      {"ADS-B polled every 216 s", 0, 216 * kS, 216 * kS},
  };
  for (const Case& c : cases) {
    Harness r(guarded());
    for (std::int64_t k = 0; k < 40; ++k) {
      r.scan(k * c.recv_per_scan, c.start + k * c.time_per_scan);
    }
    EXPECT_EQ(r.p().timeline_rejected(), 0U) << c.what;
    EXPECT_EQ(r.p().timeline_switches(), 0U) << c.what;
    EXPECT_EQ(*r.p().engine().time(), Timestamp{c.start + 39 * c.time_per_scan}) << c.what;
    r.expect_replays_identically();
  }
}

TEST(TimelineGuard, AFifteenSecondOutageSurvivesPacedAndUnpaced) {
  {
    Harness r(guarded());  // real time: the receive time covers the jump
    r.scans(0, 10);
    r.scans(25, 35);
    EXPECT_EQ(r.p().timeline_switches(), 0U);
    EXPECT_EQ(r.p().timeline_rejected(), 0U);
  }
  {
    Harness r(guarded());  // --speed 0: the jump is confirmed instead, and no frame is lost
    for (std::int64_t k = 0; k < 10; ++k) {
      r.scan(k, k * kS);
    }
    for (std::int64_t k = 25; k < 35; ++k) {
      r.scan(k, k * kS);
    }
    EXPECT_EQ(*r.p().engine().time(), Timestamp{9 * kS});
    r.tick(3 * kS);
    EXPECT_EQ(r.p().timeline_switches(), 1U);
    EXPECT_EQ(r.p().timeline_rejected(), 0U);
    r.tick(3 * kS + 1);
    EXPECT_EQ(*r.p().engine().time(), Timestamp{34 * kS});
    r.expect_replays_identically();
  }
}

TEST(TimelineGuard, ClockGateKeepsItsSemanticsWithTheGuardOn) {
  PipelineConfig c = guarded();
  c.max_time_ahead_s = 2.0;
  Harness r(c);
  r.scans(0, 5);
  r.send(5 * kS, frame(1, 777, kForged));
  r.send(5 * kS, frame(1, 777, kForged));
  EXPECT_EQ(r.p().future_rejected(), 1U);
  EXPECT_EQ(r.p().duplicates(), 1U);
  EXPECT_EQ(r.p().timeline_held(), 0U);
}

TEST(TimelineGuard, EveryFrameIsAccountedForExactlyOnce) {
  Harness r(guarded());
  r.scans(0, 6);
  r.send(6 * kS, frame(1, 1000, kForged));
  r.send(6 * kS, frame(1, 1000, kForged));
  r.send(6 * kS, frame(1, 1001, kForged + kS));
  r.send(6 * kS, frame(99, 1002, 6 * kS));
  r.send(6 * kS, frame(1, 0, 0));  // an exact repeat of the first frame
  r.finish(7 * kS);
  EXPECT_EQ(
      r.p().engine().measurements_processed() + r.p().duplicates() + r.p().timeline_rejected(),
      r.p().frames());
}

// ---- defect 2: silence ---------------------------------------------------------

std::string feed_state(const std::string& json) {
  const std::string key = "\"feed\":{\"state\":\"";
  const auto at = json.find(key);
  return at == std::string::npos
             ? ""
             : json.substr(at + key.size(), json.find('"', at + key.size()) - at - key.size());
}

// Ticks the way the daemon does until `until`, at most one per `period`.
void idle_until(Harness& r, std::int64_t from, std::int64_t until, std::int64_t period = kS) {
  std::int64_t last = from - period;
  for (std::int64_t t = from; t <= until; t += kS / 10) {
    const auto due = r.p().next_idle_tick(period);
    if (due && t >= due->ns && t - last >= period) {
      r.tick(t);
      last = t;
    }
  }
}

TEST(Silence, AnOutageIsReportedWithinTheBoundAndTracksCoastThenDrop) {
  Harness r(watched());
  r.scans(0, 15);
  const std::size_t before = r.published().size();
  ASSERT_EQ(feed_state(r.published().back()), "live");
  idle_until(r, 15 * kS, 40 * kS);
  ASSERT_GT(r.published().size(), before);
  const std::string& first = r.published()[before];
  EXPECT_EQ(feed_state(first), "stale");
  // Reported at max(coast_after 2.5 s, 3 x the 1 s cadence) + at most one idle tick.
  EXPECT_NE(first.find("\"predicted_ms\":"), std::string::npos);
  EXPECT_NE(first.find("\"status\":\"coasting\""), std::string::npos);
  EXPECT_EQ(first.find("\"status\":\"confirmed\""), std::string::npos);
  // After drop_after_s nothing is presented as a track any more.
  EXPECT_NE(r.published().back().find("\"tracks\":[]"), std::string::npos);
  r.expect_replays_identically();
}

TEST(Silence, IdleTicksNeverChangeWhatIsTracked) {
  auto measurements = [](bool idle) {
    Harness r(watched());
    r.scans(0, 15);
    if (idle) {
      idle_until(r, 15 * kS, 23 * kS);
    }
    r.scans(23, 40);
    std::istringstream in(r.out());
    std::string kept;
    for (std::string line; std::getline(in, line);) {
      if (line[0] == 'm') {
        kept += line + "\n";
      }
    }
    return kept;
  };
  EXPECT_EQ(measurements(true), measurements(false));
}

TEST(Silence, IdleTicksStopSoASilentRunStopsGrowingItsLog) {
  Harness r(watched());
  r.scans(0, 15);
  idle_until(r, 15 * kS, 60 * kS);
  const std::size_t size = r.log().size();
  EXPECT_FALSE(r.p().next_idle_tick(kS).has_value());
  idle_until(r, 60 * kS, 3'600 * kS);
  EXPECT_EQ(r.log().size(), size);
}

TEST(Silence, LateDataEndsThePredictionEvenThoughTheClockStaysPut) {
  // Found by fuzz_engine: the daemon kept being asked for idle ticks for ever.
  Harness r(watched());
  r.scans(0, 15);
  idle_until(r, 15 * kS, 20 * kS);
  ASSERT_EQ(feed_state(r.published().back()), "stale");
  const auto clock = r.p().engine().time();
  r.send(20 * kS, frame(1, 900, 13 * kS));  // on the timeline, but too old for the engine
  r.tick(20 * kS);
  ASSERT_EQ(r.p().engine().time(), clock);
  const auto due = r.p().next_idle_tick(kS);
  EXPECT_TRUE(!due || due->ns > 20 * kS);
}

TEST(Silence, ThresholdFollowsEachSensorsOwnCadence) {
  PipelineConfig c = watched();
  c.engine.lifecycle.coast_after_s = 60.0;
  c.engine.lifecycle.drop_after_s = 180.0;
  c.max_time_jump_s = 60.0;
  c.feed_stale_min_s = 60.0;
  Harness r(c);
  // Polled every 100 s: a 1 Hz threshold would alarm through every gap. The
  // cadence is known from the second poll on.
  std::size_t learned = 0;
  for (std::int64_t k = 0; k < 8; ++k) {
    r.scan(k * 100 * kS, k * 100 * kS);
    if (k == 1) {
      learned = r.published().size();
    }
    idle_until(r, k * 100 * kS + kS / 10, (k + 1) * 100 * kS - kS);
  }
  ASSERT_GT(r.published().size(), learned);
  for (std::size_t i = learned; i < r.published().size(); ++i) {
    EXPECT_NE(feed_state(r.published()[i]), "stale");
  }
  EXPECT_NE(r.published().back().find("\"stale_after_ms\":300000"), std::string::npos);
}

TEST(Silence, OneSilentSensorIsDegradedWhileTheOtherKeepsThePictureLive) {
  Harness r(watched());
  for (std::int64_t s = 0; s < 10; ++s) {
    r.scan(s * kS, s * kS, 1);
    r.scan(s * kS + kS / 2, s * kS + kS / 2, 2);
  }
  for (std::int64_t s = 10; s < 20; ++s) {
    r.scan(s * kS, s * kS, 1);  // sensor 2 went quiet at t = 9.5 s
  }
  EXPECT_EQ(feed_state(r.published().back()), "degraded");
  EXPECT_NE(r.published().back().find("\"id\":2,\"state\":\"stale\""), std::string::npos);
  EXPECT_NE(r.published().back().find("\"id\":1,\"state\":\"live\""), std::string::npos);
  EXPECT_EQ(r.confirmed(), 2U);  // the tracks themselves are real, not predictions
  r.expect_replays_identically();
}

TEST(Silence, ThePictureRecoversCleanlyWhenDataResumes) {
  Harness r(watched());
  r.scans(0, 15);
  idle_until(r, 15 * kS, 20 * kS);
  ASSERT_EQ(feed_state(r.published().back()), "stale");
  r.scans(20, 26);
  EXPECT_EQ(feed_state(r.published().back()), "live");
  EXPECT_EQ(r.confirmed(), 2U);
  r.expect_replays_identically();
}

TEST(Silence, AForgedFrameDoesNotCountAsData) {
  Harness r(watched());
  r.scans(0, 15);
  for (std::int64_t s = 15; s < 25; ++s) {
    // Each forged time contradicts the others, so no timeline can form from them.
    r.send(s * kS, frame(1, 5000 + static_cast<std::uint32_t>(s), kForged + s * 100 * kS));
    idle_until(r, s * kS, s * kS + 9 * kS / 10);
  }
  EXPECT_EQ(feed_state(r.published().back()), "stale");
}

TEST(Silence, SnapshotsAreUnchangedWhenTheFeatureIsOff) {
  Harness r{PipelineConfig{}};
  r.scans(0, 5);
  EXPECT_EQ(r.published().back().find("\"feed\""), std::string::npos);
  EXPECT_FALSE(r.p().next_idle_tick(kS).has_value());
}

// ---- header compatibility -------------------------------------------------------

TEST(EventLogCompat, NewParametersRoundTripAndOlderHeadersReadWithThemOff) {
  replay::RunHeader h{table(), watched()};
  h.config.max_time_jump_s = 7.5;
  h.config.timeline_confirm_frames = 5;
  h.config.timeline_confirm_s = 1.25;
  h.config.feed_stale_factor = 4.5;
  h.config.feed_stale_min_s = 30.0;
  const std::string text = replay::format_header(h);
  const auto back = replay::parse_header(text);
  ASSERT_TRUE(back.ok());
  EXPECT_TRUE(back.value() == h);

  std::string older;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) {
    if (line.rfind("param max_time_jump_s", 0) != 0 &&
        line.rfind("param timeline_confirm", 0) != 0 && line.rfind("param feed_stale", 0) != 0) {
      older += line + "\n";
    }
  }
  const auto old = replay::parse_header(older);
  ASSERT_TRUE(old.ok());
  EXPECT_EQ(old.value().config.max_time_jump_s, 0.0);
  EXPECT_EQ(old.value().config.feed_stale_factor, 0.0);
}

// ---- defect 3: crashes -----------------------------------------------------------

std::string recorded_log(std::size_t scans) {
  Harness r(guarded());
  r.scans(0, static_cast<std::int64_t>(scans));
  return r.log();
}

TEST(CrashRecovery, EveryPossibleTearOfTheFinalRecordIsRecoverable) {
  const std::string whole = recorded_log(4);
  // The last record is the tick of the last scan, 16 bytes; before it are frames.
  const std::size_t last_frame_end = whole.size() - replay::kRecordOverhead;
  const std::size_t frame_size = replay::kRecordOverhead + frame(1, 0, 0).size();
  for (std::size_t cut = last_frame_end - frame_size + 1; cut < last_frame_end; ++cut) {
    std::istringstream strict(whole.substr(0, cut));
    auto s = replay::EventLogReader::open(strict);
    ASSERT_TRUE(s.ok());
    Pipeline ps(s.value().header().sensors, s.value().header().config);
    EXPECT_FALSE(replay::replay(s.value(), ps).ok()) << "strict reading must refuse a torn log";

    std::istringstream tolerant(whole.substr(0, cut));
    auto t = replay::EventLogReader::open(tolerant, replay::TailPolicy::RecoverTorn);
    ASSERT_TRUE(t.ok());
    Pipeline pt(t.value().header().sensors, t.value().header().config);
    const auto stats = replay::replay(t.value(), pt);
    ASSERT_TRUE(stats.ok()) << cut;
    EXPECT_TRUE(stats.value().truncated);
    EXPECT_EQ(stats.value().torn_bytes, cut - (last_frame_end - frame_size));
  }
}

TEST(CrashRecovery, ACleanLogIsNotReportedAsTruncated) {
  const std::string whole = recorded_log(3);
  std::istringstream in(whole);
  auto r = replay::EventLogReader::open(in, replay::TailPolicy::RecoverTorn);
  ASSERT_TRUE(r.ok());
  Pipeline p(r.value().header().sensors, r.value().header().config);
  const auto stats = replay::replay(r.value(), p);
  ASSERT_TRUE(stats.ok());
  EXPECT_FALSE(stats.value().truncated);
}

TEST(CrashRecovery, CorruptionBeforeTheFinalRecordIsStillAHardError) {
  std::string damaged = recorded_log(4);
  damaged[damaged.size() - 3 * replay::kRecordOverhead] ^= 0x40;
  std::istringstream in(damaged);
  auto r = replay::EventLogReader::open(in, replay::TailPolicy::RecoverTorn);
  ASSERT_TRUE(r.ok());
  Pipeline p(r.value().header().sensors, r.value().header().config);
  const auto stats = replay::replay(r.value(), p);
  ASSERT_FALSE(stats.ok());
  EXPECT_EQ(stats.error(), replay::LogError::BadChecksum);
}

TEST(CrashRecovery, ATornHeaderIsNeverRecoverable) {
  const std::string whole = recorded_log(1);
  std::istringstream in(whole.substr(0, 40));
  EXPECT_EQ(replay::EventLogReader::open(in, replay::TailPolicy::RecoverTorn).error(),
            replay::LogError::Truncated);
}

class TempFile {
 public:
  TempFile() : path_("sifted_durable_test_" + std::to_string(::getpid()) + ".log") {}
  ~TempFile() { (void)std::remove(path_.c_str()); }
  const std::string& path() const { return path_; }
  std::size_t size() const {
    std::ifstream f(path_, std::ios::binary | std::ios::ate);
    return static_cast<std::size_t>(f.tellg());
  }

 private:
  std::string path_;
};

TEST(DurableFile, HandsRecordsToTheOsWithinTheRecordAndTimeBounds) {
  const TempFile file;
  DurableFile::Policy policy;
  policy.max_records = 3;
  policy.max_age = std::chrono::milliseconds(100);
  policy.sync_interval = std::chrono::milliseconds(0);
  auto created = DurableFile::create(file.path(), policy);
  ASSERT_TRUE(created.ok());
  DurableFile& d = *created.value();
  std::ostream os(&d);
  const auto t0 = DurableFile::Clock::now();
  os.write("ab", 2);
  d.record_written(t0);
  os.write("cd", 2);
  d.record_written(t0);
  EXPECT_EQ(file.size(), 0U);  // buffered, within both bounds
  os.write("ef", 2);
  d.record_written(t0);
  EXPECT_EQ(file.size(), 6U);  // the record bound
  os.write("gh", 2);
  d.record_written(t0);
  d.poll(t0 + std::chrono::milliseconds(99));
  EXPECT_EQ(file.size(), 6U);
  d.poll(t0 + std::chrono::milliseconds(100));
  EXPECT_EQ(file.size(), 8U);  // the time bound
}

TEST(DurableFile, AFullDiskIsReportedAndDoesNotThrow) {
  if (::access("/dev/full", W_OK) != 0) {
    GTEST_SKIP() << "no /dev/full on this platform";
  }
  auto created = DurableFile::create("/dev/full", DurableFile::Policy{});
  ASSERT_TRUE(created.ok());
  DurableFile& d = *created.value();
  std::ostream os(&d);
  replay::EventLogWriter writer(os, replay::RunHeader{table(), guarded()});
  EXPECT_FALSE(d.flush_to_os());
  EXPECT_TRUE(d.failed());
  EXPECT_FALSE(d.error().empty());
  EXPECT_FALSE(writer.write_tick(Timestamp{1}));  // later records fail quietly
}

TEST(WebSocketHeartbeat, RepublishDoesNothingBeforeTheFirstSnapshot) {
  WebSocketConfig cfg;
  cfg.port = 0;
  auto ws = WebSocketServer::start(cfg);
  ASSERT_TRUE(ws.ok());
  ws.value()->republish();  // must not crash or invent a message
  EXPECT_EQ(ws.value()->stats().messages_sent.load(), 0U);
}

}  // namespace
}  // namespace sifted::net
