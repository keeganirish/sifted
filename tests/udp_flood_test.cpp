// Floods and malformed bursts against the live receive path (M3 fault
// injection): per-source rate limiting (ADR 0010), the bounded queue, and the
// rule that every datagram is counted exactly once. Also runs under TSan: the
// receive thread and the consuming "engine" thread run concurrently.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>
#include <vector>

#include "sifted/net/impairment.hpp"
#include "sifted/net/ingest.hpp"
#include "sifted/net/rate_limit.hpp"
#include "sifted/net/system_clock.hpp"
#include "sifted/net/udp.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::net {
namespace {

constexpr std::int64_t kS = 1'000'000'000;

TEST(RateLimit, TokenBucketAdmitsBurstThenRate) {
  SourceRateLimiter l({10.0, 5.0, 16});
  int admitted = 0;
  for (int i = 0; i < 20; ++i) {
    admitted += l.admit(1, Timestamp{0}) ? 1 : 0;
  }
  EXPECT_EQ(admitted, 5);  // the burst
  // 0.5 s later: 5 more tokens.
  admitted = 0;
  for (int i = 0; i < 20; ++i) {
    admitted += l.admit(1, Timestamp{kS / 2}) ? 1 : 0;
  }
  EXPECT_EQ(admitted, 5);
  // Never more than the burst, however long the source was quiet.
  admitted = 0;
  for (int i = 0; i < 20; ++i) {
    admitted += l.admit(1, Timestamp{100 * kS}) ? 1 : 0;
  }
  EXPECT_EQ(admitted, 5);
}

TEST(RateLimit, SourcesAreIndependent) {
  SourceRateLimiter l({1.0, 2.0, 16});
  EXPECT_TRUE(l.admit(1, Timestamp{0}));
  EXPECT_TRUE(l.admit(1, Timestamp{0}));
  EXPECT_FALSE(l.admit(1, Timestamp{0}));
  EXPECT_TRUE(l.admit(2, Timestamp{0}));  // the flood from 1 does not cost 2 anything
}

TEST(RateLimit, TableIsBoundedAndEvictsTheLeastRecent) {
  SourceRateLimiter l({1.0, 1.0, 4});
  for (std::uint64_t s = 0; s < 1000; ++s) {
    EXPECT_TRUE(l.admit(s, Timestamp{static_cast<std::int64_t>(s)}));
    EXPECT_LE(l.sources(), 4U);
  }
  EXPECT_EQ(l.evictions(), 996U);
}

TEST(RateLimit, DisabledAdmitsEverythingAndInvalidIsRefused) {
  SourceRateLimiter off(RateLimitConfig{});
  for (int i = 0; i < 1000; ++i) {
    EXPECT_TRUE(off.admit(7, Timestamp{0}));
  }
  EXPECT_EQ(off.sources(), 0U);
  EXPECT_FALSE(is_valid(RateLimitConfig{10.0, 0.5, 4}));  // burst < 1
  EXPECT_FALSE(is_valid(RateLimitConfig{-1.0, 5.0, 4}));
  EXPECT_FALSE(is_valid(RateLimitConfig{10.0, 5.0, 0}));
}

// ---- against real sockets ---------------------------------------------------

class Sender {
 public:
  explicit Sender(const char* from) : fd_(::socket(AF_INET, SOCK_DGRAM, 0)) {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    ::inet_pton(AF_INET, from, &a.sin_addr);
    bound_ = ::bind(fd_, reinterpret_cast<sockaddr*>(&a), sizeof a) == 0;  // NOLINT
  }
  ~Sender() { ::close(fd_); }
  Sender(const Sender&) = delete;
  Sender& operator=(const Sender&) = delete;
  bool bound() const { return bound_; }
  void send(std::uint16_t port, const std::vector<std::uint8_t>& bytes) const {
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &to.sin_addr);
    (void)::sendto(fd_, bytes.data(), bytes.size(), 0, reinterpret_cast<sockaddr*>(&to),  // NOLINT
                   sizeof to);
  }

 private:
  int fd_;
  bool bound_ = false;
};

std::vector<std::uint8_t> frame(SensorId sensor, std::uint32_t seq, std::int64_t t) {
  Measurement m;
  m.sensor_id = sensor;
  m.kind = SensorKind::RangeBearing;
  m.seq = seq;
  m.time = Timestamp{t};
  m.z = {3000.0 + seq % 100, 0.25, 0.0};
  m.sigma = {10.0, 0.0035, 0.0};
  return wire::encode(m);
}

struct Harness {
  UdpReceiver rx;
  SystemClock clock;
  BoundedQueue<Frame> queue;
  UdpStats stats;
  std::atomic<bool> stop{false};
  std::thread thread;

  Harness(UdpReceiver r, std::size_t capacity) : rx(std::move(r)), queue(capacity) {}
  void start() {
    thread = std::thread([this] { rx.run(stop, clock, queue, stats, 10); });
  }
  void finish() {
    stop = true;
    thread.join();
  }
  std::uint64_t accounted() const {
    return stats.oversize + stats.rate_limited + stats.rejected + stats.enqueued + stats.queue_full;
  }
};

// Waits until the receiver has accounted for `n` datagrams or a deadline.
void await_accounted(const Harness& h, std::uint64_t n) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (h.accounted() < n && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

TEST(UdpFlood, DuplicateFloodFromOneSourceDoesNotStarveAnother) {
  auto bound = UdpReceiver::bind("127.0.0.1", 0);
  ASSERT_TRUE(bound.ok());
  Harness h(std::move(bound.value()), 4096);
  h.rx.set_rate_limit({200.0, 50.0, 16});
  const std::uint16_t port = h.rx.port();
  h.start();

  Sender flooder("127.0.0.1");
  Sender honest("127.0.0.2");
  ASSERT_TRUE(flooder.bound());
  if (!honest.bound()) {
    h.finish();
    GTEST_SKIP() << "cannot bind 127.0.0.2 here";
  }

  // The engine thread: consume and process concurrently with the flood.
  SensorTable table;
  ASSERT_FALSE(table.add({1, SensorKind::RangeBearing, {0.0, 0.0, 0.0}}));
  ASSERT_FALSE(table.add({2, SensorKind::RangeBearing, {0.0, 0.0, 0.0}}));
  Pipeline pipeline(table, PipelineConfig{});
  std::atomic<bool> done{false};
  std::uint64_t consumed = 0;
  std::uint64_t honest_seen = 0;
  std::thread engine([&] {
    while (!done) {
      if (auto f = h.queue.pop_for(std::chrono::milliseconds(5))) {
        ++consumed;
        honest_seen += f->bytes[8] == 2 ? 1U : 0U;
        pipeline.on_frame(f->recv, std::span<const std::uint8_t>(f->bytes.data(), f->size));
      }
    }
  });

  const auto dup = frame(1, 7, kS);  // one datagram, replayed 20000 times
  constexpr int kFlood = 20000;
  constexpr int kHonest = 40;
  for (int i = 0; i < kFlood; ++i) {
    flooder.send(port, dup);
    if (i % (kFlood / kHonest) == 0) {
      honest.send(port, frame(2, static_cast<std::uint32_t>(i), kS + i));
    }
  }
  await_accounted(h, kFlood + kHonest);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  done = true;
  engine.join();
  h.finish();

  // Some datagrams may be lost in the kernel before we see them; every one we
  // did see is counted exactly once.
  EXPECT_EQ(h.accounted(), h.stats.received.load());
  EXPECT_GT(h.stats.rate_limited.load(), kFlood / 2);
  EXPECT_LE(h.queue.size(), h.queue.capacity());
  // The honest source is far below its rate: none of its datagrams that
  // reached us was rate-limited, and all were consumed.
  EXPECT_EQ(consumed, h.stats.enqueued.load());
  EXPECT_GE(honest_seen, kHonest * 9 / 10);
  // The flood's admitted copies were suppressed as duplicates, not processed.
  EXPECT_EQ(pipeline.duplicates() + 1 + honest_seen, consumed);
}

TEST(UdpFlood, MalformedBurstIsRejectedAndNeverQueued) {
  auto bound = UdpReceiver::bind("127.0.0.1", 0);
  ASSERT_TRUE(bound.ok());
  Harness h(std::move(bound.value()), 64);
  h.start();
  Sender s("127.0.0.1");
  SplitMix64 rng(99);
  constexpr int kBurst = 5000;
  std::uint64_t oversize = 0;
  for (int i = 0; i < kBurst; ++i) {
    // Random length (some longer than any valid frame), random bytes, and
    // every tenth one a valid header with a corrupted payload.
    const std::size_t len = static_cast<std::size_t>(rng.below(200));
    std::vector<std::uint8_t> d(len);
    for (auto& b : d) {
      b = static_cast<std::uint8_t>(rng.next());
    }
    if (i % 10 == 0) {
      d = frame(1, static_cast<std::uint32_t>(i), kS);
      d[11] = 0x5A;  // the payload's reserved byte must be zero
    }
    oversize += d.size() > wire::kHeaderSize + wire::kMaxPayloadSize ? 1U : 0U;
    s.send(h.rx.port(), d);
  }
  await_accounted(h, kBurst);
  h.finish();
  EXPECT_EQ(h.accounted(), h.stats.received.load());
  EXPECT_EQ(h.stats.enqueued.load(), 0U);
  EXPECT_EQ(h.queue.size(), 0U);
  EXPECT_LE(h.stats.oversize.load(), oversize);
  EXPECT_GT(h.stats.rejected.load(), static_cast<std::uint64_t>(kBurst) / 2);
}

TEST(UdpFlood, QueueBoundHoldsWhenTheEngineIsSlow) {
  // No consumer at all: the queue fills to capacity, the rest is counted as
  // queue_full, and memory stays bounded.
  auto bound = UdpReceiver::bind("127.0.0.1", 0);
  ASSERT_TRUE(bound.ok());
  Harness h(std::move(bound.value()), 100);
  h.start();
  Sender s("127.0.0.1");
  for (std::uint32_t i = 0; i < 3000; ++i) {
    s.send(h.rx.port(), frame(1, i, kS + i));
  }
  await_accounted(h, 3000);
  h.finish();
  EXPECT_EQ(h.accounted(), h.stats.received.load());
  EXPECT_EQ(h.queue.size(), 100U);
  EXPECT_EQ(h.stats.enqueued.load(), 100U);
  EXPECT_EQ(h.stats.queue_full.load(), h.stats.received.load() - 100U);
}

}  // namespace
}  // namespace sifted::net
