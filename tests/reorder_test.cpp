// Ingest reorder stage and clock gate (ADRs 0009, 0010), and that both are
// recorded so a replay stays bit for bit.

#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "sifted/net/impairment.hpp"
#include "sifted/net/ingest.hpp"
#include "sifted/replay/event_log.hpp"
#include "sifted/replay/replayer.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::net {
namespace {

constexpr std::int64_t kS = 1'000'000'000;

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

struct Capture {
  std::vector<std::string> lines;
  Pipeline::LineSink sink() {
    return [this](std::string_view l) { lines.emplace_back(l); };
  }
  // The outcome token of every measurement line, in output order.
  std::vector<std::string> outcomes() const {
    std::vector<std::string> out;
    for (const auto& l : lines) {
      if (l[0] == 'm') {
        std::istringstream s(l);
        std::string tok;
        for (int i = 0; i < 6; ++i) {
          s >> tok;
        }
        out.push_back(tok);
      }
    }
    return out;
  }
  std::vector<std::int64_t> times() const {
    std::vector<std::int64_t> out;
    for (const auto& l : lines) {
      if (l[0] == 'm') {
        std::istringstream s(l);
        std::string tok;
        std::int64_t t = 0;
        s >> tok >> tok >> tok >> tok >> t;
        out.push_back(t);
      }
    }
    return out;
  }
};

PipelineConfig with_window(double s) {
  PipelineConfig c;
  c.reorder_window_s = s;
  return c;
}

TEST(Reorder, DisabledKeepsArrivalOrderAndDropsLateFrames) {
  Capture cap;
  Pipeline p(table(), PipelineConfig{}, cap.sink());
  p.on_frame(Timestamp{2 * kS}, frame(1, 1, 2 * kS));
  p.on_frame(Timestamp{2 * kS + 10}, frame(1, 0, 1 * kS));  // overtaken
  p.finish(Timestamp{3 * kS});
  EXPECT_EQ(cap.outcomes(), (std::vector<std::string>{"initialized", "out_of_order"}));
}

TEST(Reorder, WindowRestoresMeasurementOrder) {
  Capture cap;
  Pipeline p(table(), with_window(0.5), cap.sink());
  p.on_frame(Timestamp{2 * kS}, frame(1, 1, 2 * kS));
  p.on_frame(Timestamp{2 * kS + 10}, frame(1, 0, 1 * kS));
  EXPECT_TRUE(p.has_pending());
  EXPECT_EQ(p.held(), 2U);
  p.on_tick(Timestamp{2 * kS + kS / 2 + 10});  // both holds expired
  EXPECT_EQ(cap.times(), (std::vector<std::int64_t>{1 * kS, 2 * kS}));
  EXPECT_EQ(cap.outcomes(), (std::vector<std::string>{"initialized", "updated"}));
  EXPECT_EQ(p.engine().count(MeasurementOutcome::OutOfOrder), 0U);
}

TEST(Reorder, ReleaseWaitsForTheWindowAndKeepsLaterTimesBack) {
  Capture cap;
  Pipeline p(table(), with_window(1.0), cap.sink());
  p.on_frame(Timestamp{10 * kS}, frame(1, 0, 10 * kS));
  p.on_frame(Timestamp{10 * kS + kS / 2}, frame(1, 1, 11 * kS));
  p.on_tick(Timestamp{10 * kS + kS - 1});
  EXPECT_TRUE(cap.times().empty());  // nothing has waited a full second
  p.on_tick(Timestamp{11 * kS});     // the first has: it is released, not the second
  EXPECT_EQ(cap.times(), (std::vector<std::int64_t>{10 * kS}));
  EXPECT_EQ(p.held(), 1U);
  p.finish(Timestamp{11 * kS});  // finish releases everything
  EXPECT_EQ(cap.times(), (std::vector<std::int64_t>{10 * kS, 11 * kS}));
  EXPECT_FALSE(p.has_pending());
}

TEST(Reorder, ReleasesHeldFramesWithEarlierTimesTogetherWithAnExpiredOne) {
  // An expired frame releases every held frame with a measurement time at or
  // before its own, even ones received later, so released times never go back.
  Capture cap;
  Pipeline p(table(), with_window(1.0), cap.sink());
  p.on_frame(Timestamp{10 * kS}, frame(1, 2, 12 * kS));
  p.on_frame(Timestamp{10 * kS + 100}, frame(2, 0, 11 * kS, 4100.0));
  p.on_tick(Timestamp{11 * kS});  // only the first has expired; W = 12 s
  EXPECT_EQ(cap.times(), (std::vector<std::int64_t>{11 * kS, 12 * kS}));
}

TEST(Reorder, LateFrameBypassesTheStageAndIsCounted) {
  Capture cap;
  Pipeline p(table(), with_window(0.2), cap.sink());
  p.on_frame(Timestamp{5 * kS}, frame(1, 0, 5 * kS));
  p.on_tick(Timestamp{5 * kS + kS / 2});
  p.on_frame(Timestamp{6 * kS}, frame(1, 1, 4 * kS));  // older than what was released
  EXPECT_EQ(p.reorder_late(), 1U);
  p.finish(Timestamp{6 * kS});
  EXPECT_EQ(cap.outcomes(), (std::vector<std::string>{"initialized", "out_of_order"}));
}

TEST(Reorder, SplitScanIsReassembledIntoOneGroup) {
  // Two scans of two detections each; jitter interleaves them. Without the
  // stage the second scan's first detection makes the first scan's remainder
  // OutOfOrder; with it, both scans are associated whole.
  const std::vector<std::pair<std::int64_t, std::vector<std::uint8_t>>> arrivals = {
      {1 * kS + 1, frame(1, 0, 1 * kS, 4000.0)},
      {2 * kS + 1, frame(1, 2, 2 * kS, 4000.0)},
      {2 * kS + 2, frame(1, 1, 1 * kS, 5000.0)},  // delayed past the next scan
      {2 * kS + 3, frame(1, 3, 2 * kS, 5000.0)},
  };
  for (const double window : {0.0, 1.5}) {
    Capture cap;
    Pipeline p(table(), with_window(window), cap.sink());
    for (const auto& [t, f] : arrivals) {
      p.on_frame(Timestamp{t}, f);
    }
    p.finish(Timestamp{4 * kS});
    const auto ooo = p.engine().count(MeasurementOutcome::OutOfOrder);
    if (window == 0.0) {
      EXPECT_EQ(ooo, 1U);
    } else {
      EXPECT_EQ(ooo, 0U);
      EXPECT_EQ(p.scans(), 2U);
      EXPECT_EQ(p.engine().tracks_started(), 2U);
    }
  }
}

TEST(Reorder, CapacityBoundsTheStage) {
  PipelineConfig c = with_window(60.0);
  c.reorder_capacity = 8;
  Capture cap;
  Pipeline p(table(), c, cap.sink());
  for (std::uint32_t i = 0; i < 100; ++i) {
    p.on_frame(Timestamp{kS + i}, frame(1, i, kS + static_cast<std::int64_t>(i) * 1000));
    EXPECT_LE(p.held(), 8U);
  }
  EXPECT_EQ(p.held_high_water(), 9U);  // momentarily, before the forced release
  EXPECT_EQ(p.reorder_forced(), 92U);
  p.finish(Timestamp{2 * kS});
  EXPECT_EQ(p.engine().measurements_processed(), 100U);
}

TEST(Reorder, DuplicatesAreSuppressedBeforeTheStage) {
  Capture cap;
  Pipeline p(table(), with_window(0.5), cap.sink());
  p.on_frame(Timestamp{kS}, frame(1, 0, kS));
  p.on_frame(Timestamp{kS + 5}, frame(1, 0, kS));
  EXPECT_EQ(p.held(), 1U);
  EXPECT_EQ(p.duplicates(), 1U);
}

TEST(ClockGate, RejectsFramesTooFarAheadOfTheirReceiveTime) {
  PipelineConfig c;
  c.max_time_ahead_s = 1.0;
  Capture cap;
  Pipeline p(table(), c, cap.sink());
  p.on_frame(Timestamp{10 * kS}, frame(1, 0, 10 * kS + kS / 2));  // 0.5 s ahead: fine
  p.on_frame(Timestamp{10 * kS}, frame(2, 0, 15 * kS));           // 5 s ahead: rejected
  p.on_frame(Timestamp{10 * kS}, frame(2, 1, 3 * kS));            // behind is not judged here
  p.finish(Timestamp{11 * kS});
  EXPECT_EQ(p.future_rejected(), 1U);
  ASSERT_GE(cap.lines.size(), 2U);
  EXPECT_EQ(cap.lines[0].substr(0, 2), "f ");
  // Without the gate the 15 s frame would have moved the engine clock.
  EXPECT_EQ(p.engine().time(), Timestamp{10 * kS + kS / 2});
}

TEST(ClockGate, ExtremeTimesDoNotOverflow) {
  PipelineConfig c;
  c.max_time_ahead_s = 1.0;
  c.reorder_window_s = 0.5;
  Pipeline p(table(), c);
  p.on_frame(Timestamp{INT64_MIN}, frame(1, 0, INT64_MAX));
  p.on_frame(Timestamp{INT64_MAX}, frame(1, 1, INT64_MIN));
  p.on_tick(Timestamp{INT64_MAX});
  p.finish(Timestamp{INT64_MAX});
  EXPECT_EQ(p.future_rejected(), 1U);
}

TEST(Reorder, NewParametersAreRecordedAndOldHeadersStillParse) {
  replay::RunHeader h;
  h.sensors = table();
  h.config.reorder_window_s = 0.375;
  h.config.reorder_capacity = 77;
  h.config.max_time_ahead_s = 2.5;
  h.config.engine.association.method = association::AssignmentMethod::Greedy;
  const std::string text = replay::format_header(h);
  EXPECT_NE(text.find("param reorder_window_s 0.375\n"), std::string::npos);
  EXPECT_NE(text.find("param assignment greedy\n"), std::string::npos);
  const auto back = replay::parse_header(text);
  ASSERT_TRUE(back.ok());
  EXPECT_EQ(back.value(), h);

  // An M2 header (none of the M3 keys) reads with defaults that reproduce M2.
  std::string old;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) {
    if (line.find("reorder_") == std::string::npos &&
        line.find("max_time_ahead") == std::string::npos &&
        line.find("param assignment ") == std::string::npos) {
      old += line + "\n";
    }
  }
  const auto m2 = replay::parse_header(old);
  ASSERT_TRUE(m2.ok());
  EXPECT_EQ(m2.value().config.reorder_window_s, 0.0);
  EXPECT_EQ(m2.value().config.max_time_ahead_s, 0.0);
  EXPECT_EQ(m2.value().config.engine.association.method,
            association::AssignmentMethod::GlobalNearestNeighbour);
  // But an unknown key or a bad value is still refused.
  EXPECT_FALSE(replay::parse_header("sifted-log 1\nparam assignment fastest\n" +
                                    old.substr(old.find('\n') + 1))
                   .ok());
}

// An impaired run, recorded the way the daemon records it (every frame and
// tick the engine thread consumed, with its receive time), replays to the
// same output, byte for byte, with the reorder stage and the clock gate on.
TEST(Reorder, ImpairedRecordedRunReplaysBitForBit) {
  const auto sc = testing::load_scenario("enter_leave");
  ImpairmentConfig ic;
  ic.seed = 3;
  ic.global.loss = 0.05;
  ic.global.p_good_to_bad = 0.01;
  ic.global.p_bad_to_good = 0.3;
  ic.global.jitter_ns = 400'000'000;
  ic.global.duplicate = 0.05;
  ic.outages = {{50 * kS, 58 * kS}};
  const auto arrivals = impair(testing::sent_frames(sc.measurements), ic);

  replay::RunHeader h;
  h.sensors = sc.sensors;
  h.config.reorder_window_s = 0.5;
  h.config.max_time_ahead_s = 2.0;
  std::ostringstream log;
  replay::EventLogWriter writer(log, h);
  std::string live;
  Pipeline p(h.sensors, h.config, [&](std::string_view l) { live.append(l); });
  // Daemon-like consumption: frames, plus a tick after 100 ms of quiet.
  std::optional<Timestamp> last;
  for (const auto& a : arrivals) {
    while (last && p.has_pending() && last->ns + testing::kDaemonTickNs < a.arrival.ns) {
      last = Timestamp{last->ns + testing::kDaemonTickNs};
      ASSERT_TRUE(writer.write_tick(*last));
      p.on_tick(*last);
    }
    ASSERT_TRUE(writer.write_frame(a.arrival, a.bytes));
    p.on_frame(a.arrival, a.bytes);
    last = a.arrival;
  }
  p.finish(*last);
  EXPECT_GT(p.engine().count(MeasurementOutcome::Updated), 500U);

  std::istringstream in(log.str());
  auto reader = replay::EventLogReader::open(in);
  ASSERT_TRUE(reader.ok());
  EXPECT_EQ(reader.value().header(), h);
  std::string replayed;
  Pipeline q(reader.value().header().sensors, reader.value().header().config,
             [&](std::string_view l) { replayed.append(l); });
  ASSERT_TRUE(replay::replay(reader.value(), q).ok());
  EXPECT_EQ(replayed, live);
  EXPECT_FALSE(live.empty());
}

}  // namespace
}  // namespace sifted::net
