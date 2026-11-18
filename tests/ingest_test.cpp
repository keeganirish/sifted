// Ingest: duplicate suppression, scan assembly, snapshots, the bounded queue
// (including a concurrent producer test for ThreadSanitizer) and UDP loopback.

#include "sifted/net/ingest.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "sifted/core/clock.hpp"
#include "sifted/net/bounded_queue.hpp"
#include "sifted/net/dedup.hpp"
#include "sifted/net/snapshot.hpp"
#include "sifted/net/system_clock.hpp"
#include "sifted/net/udp.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::net {
namespace {

SensorTable table() {
  SensorTable t;
  EXPECT_FALSE(t.add({1, SensorKind::RangeBearing, {0.0, 0.0, 0.0}}));
  EXPECT_FALSE(t.add({2, SensorKind::RangeBearing, {100.0, 0.0, 0.0}}));
  return t;
}

Measurement meas(SensorId s, std::uint32_t seq, std::int64_t t, double range = 4000.0,
                 double bearing = 0.3) {
  Measurement m;
  m.sensor_id = s;
  m.kind = SensorKind::RangeBearing;
  m.seq = seq;
  m.time = Timestamp{t};
  m.z = {range, bearing, 0.0};
  m.sigma = {10.0, 0.0035, 0.0};
  return m;
}

std::vector<std::string> lines_of(const std::string& s) {
  std::vector<std::string> out;
  std::size_t b = 0;
  while (b < s.size()) {
    const std::size_t e = s.find('\n', b);
    out.push_back(s.substr(b, e - b));
    b = e + 1;
  }
  return out;
}

}  // namespace

TEST(DuplicateFilter, SuppressesRepeatsPerSensorAndToleratesRestarts) {
  const SensorTable t = table();
  DuplicateFilter f(t, 4);
  EXPECT_FALSE(f.is_duplicate(1, 10, Timestamp{100}));
  EXPECT_TRUE(f.is_duplicate(1, 10, Timestamp{100}));   // exact repeat
  EXPECT_FALSE(f.is_duplicate(2, 10, Timestamp{100}));  // other sensor
  EXPECT_FALSE(f.is_duplicate(1, 10, Timestamp{900}));  // restarted sensor: same seq, new time
  EXPECT_FALSE(f.is_duplicate(9, 1, Timestamp{1}));     // unknown sensor: not tracked
  EXPECT_FALSE(f.is_duplicate(9, 1, Timestamp{1}));
  for (std::uint32_t s = 11; s < 15; ++s) {
    EXPECT_FALSE(f.is_duplicate(1, s, Timestamp{100 + s}));
  }
  EXPECT_FALSE(f.is_duplicate(1, 10, Timestamp{100}));  // aged out of the 4-key window
  EXPECT_EQ(f.suppressed(), 1U);
}

TEST(Pipeline, AssemblesScansByConsecutiveSensorAndTime) {
  std::string out;
  Pipeline p(table(), {}, [&](std::string_view l) { out.append(l); });
  // Two detections in one scan (sensor 1, t=0), then sensor 2 at the same time.
  p.on_frame(Timestamp{1}, wire::encode(meas(1, 0, 0, 4000.0, 0.3)));
  p.on_frame(Timestamp{2}, wire::encode(meas(1, 1, 0, 4000.0, -0.3)));
  EXPECT_TRUE(p.has_pending());
  EXPECT_EQ(p.scans(), 0U);
  p.on_frame(Timestamp{3}, wire::encode(meas(2, 0, 0, 5000.0, 1.0)));
  EXPECT_EQ(p.scans(), 1U);  // the sensor-1 scan was flushed as one group
  p.on_tick(Timestamp{4});
  EXPECT_EQ(p.scans(), 2U);
  EXPECT_FALSE(p.has_pending());
  const auto ls = lines_of(out);
  ASSERT_GE(ls.size(), 4U);
  EXPECT_EQ(ls[0], "m 3 1 0 0 initialized 1");
  EXPECT_EQ(ls[1], "m 3 1 1 0 initialized 2");
  EXPECT_EQ(ls[2].rfind("s {\"type\":\"sifted.tracks\",\"version\":2,\"seq\":1,\"time_ns\":0", 0),
            0U);
  EXPECT_EQ(ls[3], "m 4 2 0 0 initialized 3");
}

TEST(Pipeline, ReportsDecodeErrorsAndDuplicates) {
  std::string out;
  Pipeline p(table(), {}, [&](std::string_view l) { out.append(l); });
  const auto f = wire::encode(meas(1, 5, 1000));
  p.on_frame(Timestamp{1}, f);
  p.on_frame(Timestamp{2}, f);
  std::vector<std::uint8_t> bad = f;
  bad.pop_back();
  p.on_frame(Timestamp{3}, bad);
  p.finish(Timestamp{3});
  EXPECT_EQ(p.duplicates(), 1U);
  EXPECT_EQ(p.decode_errors(wire::DecodeError::Truncated), 1U);
  const auto ls = lines_of(out);
  EXPECT_EQ(ls[0], "d 2 1 5 1000");
  EXPECT_EQ(ls[1], "x 3 truncated");
  EXPECT_EQ(ls[2], "m 3 1 5 1000 initialized 1");
}

TEST(Pipeline, SnapshotsFollowMeasurementTimeNotTheWallClock) {
  PipelineConfig cfg;
  cfg.snapshot_interval_s = 2.0;
  std::vector<std::string> published;
  Pipeline p(table(), cfg, {}, [&](const std::string& j) { published.push_back(j); });
  for (std::uint32_t k = 0; k <= 6; ++k) {
    p.on_frame(Timestamp{k}, wire::encode(meas(1, k, k * 1'000'000'000LL)));
  }
  p.finish(Timestamp{7});
  ASSERT_EQ(published.size(), 4U);  // t = 0, 2, 4, 6 s
  EXPECT_NE(published[1].find("\"time_ns\":2000000000"), std::string::npos);
}

TEST(Snapshot, FormatIsVersionedAndComplete) {
  TrackEstimate e;
  e.id = 7;
  e.status = association::TrackStatus::Coasting;
  e.state = estimation::StateVector::Zero();
  e.state(0) = 1000.0;
  e.state(1) = -500.5;
  e.covariance = estimation::StateCovariance::Identity();
  e.covariance(0, 0) = 4.0;
  e.updates = 12;
  e.last_update = Timestamp{99};
  e.sensors = {1, 20};
  const std::string j =
      format_snapshot(3, Timestamp{100}, {e}, estimation::GeoOrigin{0.0, 0.0, 0.0});
  EXPECT_EQ(j.rfind("{\"type\":\"sifted.tracks\",\"version\":2,\"seq\":3,\"time_ns\":100,", 0), 0U);
  for (const char* key : {"\"id\":7", "\"status\":\"coasting\"", "\"x\":1000", "\"y\":-500.5",
                          "\"cov\":[4,0,1]", "\"sigma_m\":2", "\"lat\":", "\"lon\":",
                          "\"updates\":12", "\"sensors\":[1,20]", "\"manoeuvring\":false"}) {
    EXPECT_NE(j.find(key), std::string::npos) << key << " in " << j;
  }
  EXPECT_EQ(format_snapshot(1, Timestamp{0}, {}, std::nullopt),
            "{\"type\":\"sifted.tracks\",\"version\":2,\"seq\":1,\"time_ns\":0,\"origin\":null,"
            "\"tracks\":[]}");
}

TEST(Snapshot, EncodesTraceEvents) {
  TrackEstimate e;
  e.id = 3;
  e.state = estimation::StateVector::Zero();
  e.covariance = estimation::StateCovariance::Identity();
  e.trace_total = 12;

  store::TraceEvent associated;
  associated.kind = store::TraceEventKind::Associated;
  associated.time = Timestamp{42};
  associated.sensor = 5;
  associated.seq = 9;
  associated.distance2 = 2.5;
  associated.gate = 13.75;
  store::TraceEvent competed = associated;
  competed.kind = store::TraceEventKind::Competed;
  competed.other = 7;
  store::TraceEvent coasting;  // no gating statistic: NaN in the store
  coasting.kind = store::TraceEventKind::Coasting;
  coasting.time = Timestamp{50};
  store::TraceEvent dropped;
  dropped.kind = store::TraceEventKind::Dropped;
  dropped.time = Timestamp{60};
  dropped.drop_reason = association::DropReason::TentativeTimeout;
  e.trace = {associated, competed, coasting, dropped};

  const std::string j = format_snapshot(1, Timestamp{60}, {e}, std::nullopt);
  const std::string expected =
      "\"trace_total\":12,\"trace\":["
      "{\"kind\":\"associated\",\"time_ns\":42,\"sensor\":5,\"seq\":9,\"d2\":2.5,\"gate\":13.75,"
      "\"other\":0,\"drop_reason\":\"none\"},"
      "{\"kind\":\"competed\",\"time_ns\":42,\"sensor\":5,\"seq\":9,\"d2\":2.5,\"gate\":13.75,"
      "\"other\":7,\"drop_reason\":\"none\"},"
      "{\"kind\":\"coasting\",\"time_ns\":50,\"sensor\":0,\"seq\":0,\"d2\":null,\"gate\":null,"
      "\"other\":0,\"drop_reason\":\"none\"},"
      "{\"kind\":\"dropped\",\"time_ns\":60,\"sensor\":0,\"seq\":0,\"d2\":null,\"gate\":null,"
      "\"other\":0,\"drop_reason\":\"tentative timeout\"}]}]}";
  ASSERT_GE(j.size(), expected.size());
  EXPECT_EQ(j.substr(j.size() - expected.size()), expected);
}

TEST(Snapshot, TraceStringsAreStableAndNeedNoEscaping) {
  const auto plain = [](std::string_view s) {
    return !s.empty() && std::all_of(s.begin(), s.end(),
                                     [](char c) { return (c >= 'a' && c <= 'z') || c == ' '; });
  };
  const std::array<std::string_view, store::kTraceEventKindCount> kinds = {
      "initiated", "associated", "competed", "near miss",       "update rejected", "confirmed",
      "coasting",  "recovered",  "dropped",  "manoeuvre start", "manoeuvre end"};
  for (std::size_t k = 0; k < kinds.size(); ++k) {
    EXPECT_EQ(store::to_string(static_cast<store::TraceEventKind>(k)), kinds[k]);
    EXPECT_TRUE(plain(kinds[k])) << kinds[k];
  }
  for (const auto r : {association::DropReason::None, association::DropReason::TentativeTimeout,
                       association::DropReason::Stale, association::DropReason::Uncertainty}) {
    EXPECT_TRUE(plain(association::to_string(r))) << association::to_string(r);
  }
}

TEST(Pipeline, TraceExcerptLengthIsAParameterAndNeverChangesTracking) {
  const auto run = [](std::size_t excerpt) {
    PipelineConfig cfg;
    cfg.snapshot_interval_s = 0.0;
    cfg.snapshot_trace_events = excerpt;
    std::string out;
    Pipeline p(table(), cfg, [&](std::string_view l) { out.append(l); });
    for (std::uint32_t k = 0; k < 12; ++k) {
      const std::int64_t t = k * 1'000'000'000LL;
      p.on_frame(Timestamp{t}, wire::encode(meas(1, 2 * k, t, 4000.0 + k, 0.3)));
      p.on_frame(Timestamp{t}, wire::encode(meas(1, 2 * k + 1, t, 4040.0 + k, 0.3)));
    }
    p.finish(Timestamp{12'000'000'000LL});
    return out;
  };
  // The output with the excerpt removed, which must not depend on its length.
  const auto without_trace = [](std::string s) {
    for (std::size_t b = s.find(",\"trace\":["); b != std::string::npos;
         b = s.find(",\"trace\":[", b)) {
      s.erase(b, s.find(']', b) + 1 - b);
    }
    return s;
  };
  const auto events_in_last_snapshot = [](const std::string& s) {
    std::size_t n = 0;
    const std::size_t last = s.rfind("\ns {");
    for (std::size_t b = s.find("\"kind\":", last); b != std::string::npos;
         b = s.find("\"kind\":", b + 1)) {
      ++n;
    }
    return n;
  };

  const std::string none = run(0);
  const std::string two = run(2);
  const std::string all = run(kMaxSnapshotTraceEvents);
  EXPECT_EQ(events_in_last_snapshot(none), 0U);
  EXPECT_EQ(events_in_last_snapshot(two), 4U);  // two tracks
  EXPECT_GT(events_in_last_snapshot(all), 4U);
  EXPECT_EQ(without_trace(two), without_trace(none));
  EXPECT_EQ(without_trace(all), without_trace(none));
  EXPECT_EQ(run(2), two);
}

TEST(BoundedQueue, DropsWhenFullAndKeepsOrder) {
  BoundedQueue<int> q(2);
  EXPECT_TRUE(q.try_push(1));
  EXPECT_TRUE(q.try_push(2));
  EXPECT_FALSE(q.try_push(3));
  EXPECT_EQ(q.dropped(), 1U);
  EXPECT_EQ(q.pop_for(std::chrono::milliseconds(0)), 1);
  EXPECT_EQ(q.pop_for(std::chrono::milliseconds(0)), 2);
  EXPECT_FALSE(q.pop_for(std::chrono::milliseconds(1)).has_value());
  q.close();
  EXPECT_FALSE(q.try_push(4));
}

TEST(BoundedQueue, ConcurrentProducersSingleConsumer) {
  BoundedQueue<std::uint64_t> q(64);
  constexpr int kProducers = 4;
  constexpr std::uint64_t kEach = 5000;
  std::atomic<int> done{0};
  std::vector<std::thread> producers;
  for (int p = 0; p < kProducers; ++p) {
    producers.emplace_back([&, p] {
      for (std::uint64_t i = 0; i < kEach; ++i) {
        q.try_push(static_cast<std::uint64_t>(p) << 32 | i);
      }
      done.fetch_add(1);
    });
  }
  std::uint64_t received = 0;
  std::vector<std::int64_t> last(kProducers, -1);
  bool ordered = true;
  while (done.load() < kProducers || q.size() > 0) {
    if (auto v = q.pop_for(std::chrono::milliseconds(5))) {
      ++received;
      const auto p = static_cast<std::size_t>(*v >> 32);
      const auto i = static_cast<std::int64_t>(*v & 0xFFFFFFFFU);
      ordered = ordered && i > last[p];  // per-producer FIFO
      last[p] = i;
    }
  }
  for (auto& t : producers) {
    t.join();
  }
  EXPECT_TRUE(ordered);
  EXPECT_EQ(received + q.dropped(), kProducers * kEach);
  EXPECT_EQ(received, q.accepted());
}

TEST(SystemClock, NeverGoesBackwards) {
  const SystemClock c;
  Timestamp prev = c.now();
  for (int i = 0; i < 1000; ++i) {
    const Timestamp t = c.now();
    ASSERT_GE(t, prev);
    prev = t;
  }
}

TEST(Udp, ReceivesValidatesAndEnqueues) {
  auto rx = UdpReceiver::bind("127.0.0.1", 0);
  ASSERT_TRUE(rx.ok()) << rx.error();
  UdpReceiver r = std::move(rx.value());
  ASSERT_NE(r.port(), 0);
  BoundedQueue<Frame> q(2);
  UdpStats stats;
  std::atomic<bool> stop{false};
  SimulatedClock clock(Timestamp{42});
  std::thread t([&] { r.run(stop, clock, q, stats, 10); });

  const int s = ::socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_port = htons(r.port());
  inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);
  auto send = [&](const std::vector<std::uint8_t>& b) {
    ::sendto(s, b.data(), b.size(), 0, reinterpret_cast<const sockaddr*>(&to),
             sizeof to);  // NOLINT
  };
  send(wire::encode(meas(1, 0, 0)));
  send({'n', 'o', 'p', 'e'});
  send(std::vector<std::uint8_t>(5000, 0));
  send(wire::encode(meas(1, 1, 0)));
  send(wire::encode(meas(1, 2, 0)));  // queue capacity 2: dropped
  for (int i = 0; i < 200 && stats.received.load() < 5; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  stop = true;
  t.join();
  ::close(s);
  EXPECT_EQ(stats.received.load(), 5U);
  EXPECT_EQ(stats.rejected.load(), 1U);
  EXPECT_EQ(stats.oversize.load(), 1U);
  EXPECT_EQ(stats.enqueued.load(), 2U);
  EXPECT_EQ(stats.queue_full.load(), 1U);
  const auto f = q.pop_for(std::chrono::milliseconds(0));
  ASSERT_TRUE(f.has_value());
  EXPECT_EQ(f->recv, Timestamp{42});
  EXPECT_EQ(f->size, wire::kMeasurementFrameSize);
}

}  // namespace sifted::net
