#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "sifted/core/clock.hpp"
#include "sifted/core/result.hpp"
#include "sifted/net/bounded_queue.hpp"
#include "sifted/net/rate_limit.hpp"
#include "sifted/wire/codec.hpp"

// UDP ingest: the network thread's whole job (docs/design.md section 2).
// Receive a datagram, drop it if its source address is over its rate limit,
// reject it unless the strict wire decoder accepts it (cheap, and keeps
// garbage out of the queue and the log), stamp it with the receive time, and
// try to enqueue it. Never touches the engine. Every datagram is counted in
// exactly one of oversize, rate_limited, rejected, enqueued, queue_full.
namespace sifted::net {

// One received frame, as the engine thread consumes it.
struct Frame {
  enum class Kind : std::uint8_t { Data, Tick };
  Kind kind = Kind::Data;
  Timestamp recv{};
  std::uint16_t size = 0;
  std::array<std::uint8_t, wire::kHeaderSize + wire::kMaxPayloadSize> bytes{};
};

struct UdpStats {
  std::atomic<std::uint64_t> received{0};
  std::atomic<std::uint64_t> oversize{0};      // longer than any valid frame
  std::atomic<std::uint64_t> rate_limited{0};  // dropped: source over its rate (ADR 0010)
  std::atomic<std::uint64_t> rejected{0};      // failed the wire decoder
  std::atomic<std::uint64_t> enqueued{0};
  std::atomic<std::uint64_t> queue_full{0};  // dropped: engine queue full
};

class UdpReceiver {
 public:
  // Binds a UDP socket to `address:port` (IPv4 dotted quad; port 0 picks an
  // ephemeral port). Returns an error message on failure.
  static Result<UdpReceiver, std::string> bind(const std::string& address, std::uint16_t port);

  UdpReceiver(UdpReceiver&& o) noexcept;
  UdpReceiver& operator=(UdpReceiver&& o) noexcept;
  UdpReceiver(const UdpReceiver&) = delete;
  UdpReceiver& operator=(const UdpReceiver&) = delete;
  ~UdpReceiver();

  std::uint16_t port() const noexcept { return port_; }

  // Per-source-address token bucket applied by run() (default: off). Call
  // before run(); throws std::invalid_argument on an invalid config.
  void set_rate_limit(const RateLimitConfig& c);

  // Receive loop; returns when `stop` becomes true (checked at least every
  // poll_ms). Run on its own thread.
  void run(const std::atomic<bool>& stop, const Clock& clock, BoundedQueue<Frame>& queue,
           UdpStats& stats, int poll_ms = 100);

 private:
  UdpReceiver(int fd, std::uint16_t port) : fd_(fd), port_(port) {}
  int fd_ = -1;
  std::uint16_t port_ = 0;
  RateLimitConfig rate_limit_;
};

}  // namespace sifted::net
