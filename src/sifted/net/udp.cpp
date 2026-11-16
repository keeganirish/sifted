#include "sifted/net/udp.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <span>
#include <utility>

namespace sifted::net {

Result<UdpReceiver, std::string> UdpReceiver::bind(const std::string& address, std::uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, address.c_str(), &addr.sin_addr) != 1) {
    return std::string("invalid IPv4 address: ") + address;
  }
  const int fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return std::string("socket: ") + std::strerror(errno);
  }
  // Best effort: a larger kernel buffer absorbs bursts (e.g. one radar scan
  // of many detections) while the receive thread is busy. Losses beyond it
  // happen in the kernel, before our counters see them.
  const int rcvbuf = 4 * 1024 * 1024;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof rcvbuf);
  if (::bind(fd, reinterpret_cast<const sockaddr*>(&addr), sizeof addr) != 0) {  // NOLINT
    const std::string err =
        std::string("bind ") + address + ":" + std::to_string(port) + ": " + std::strerror(errno);
    ::close(fd);
    return err;
  }
  socklen_t len = sizeof addr;
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len);  // NOLINT
  return UdpReceiver(fd, ntohs(addr.sin_port));
}

UdpReceiver::UdpReceiver(UdpReceiver&& o) noexcept
    : fd_(std::exchange(o.fd_, -1)), port_(o.port_), rate_limit_(o.rate_limit_) {}

void UdpReceiver::set_rate_limit(const RateLimitConfig& c) {
  SourceRateLimiter check(c);  // validates
  rate_limit_ = c;
}

UdpReceiver& UdpReceiver::operator=(UdpReceiver&& o) noexcept {
  if (this != &o) {
    if (fd_ >= 0) {
      ::close(fd_);
    }
    fd_ = std::exchange(o.fd_, -1);
    port_ = o.port_;
    rate_limit_ = o.rate_limit_;
  }
  return *this;
}

UdpReceiver::~UdpReceiver() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

void UdpReceiver::run(const std::atomic<bool>& stop, const Clock& clock, BoundedQueue<Frame>& queue,
                      UdpStats& stats, int poll_ms) {
  // One byte larger than the largest valid frame, so oversize datagrams are
  // detected rather than silently truncated to something that might decode.
  std::array<std::uint8_t, wire::kHeaderSize + wire::kMaxPayloadSize + 1> buf{};
  SourceRateLimiter limiter(rate_limit_);
  while (!stop.load(std::memory_order_relaxed)) {
    pollfd p{fd_, POLLIN, 0};
    const int ready = ::poll(&p, 1, poll_ms);
    if (ready <= 0) {
      continue;  // timeout or EINTR: re-check stop
    }
    sockaddr_in from{};
    socklen_t from_len = sizeof from;
    const ssize_t n = ::recvfrom(fd_, buf.data(), buf.size(), MSG_DONTWAIT,
                                 reinterpret_cast<sockaddr*>(&from), &from_len);  // NOLINT
    if (n < 0) {
      continue;
    }
    const Timestamp recv = clock.now();
    stats.received.fetch_add(1, std::memory_order_relaxed);
    const auto size = static_cast<std::size_t>(n);
    if (size > buf.size() - 1) {
      stats.oversize.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    // Keyed by source address only: rotating source ports buys nothing.
    if (!limiter.admit(ntohl(from.sin_addr.s_addr), recv)) {
      stats.rate_limited.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    if (!wire::decode_measurement(std::span<const std::uint8_t>(buf.data(), size)).ok()) {
      stats.rejected.fetch_add(1, std::memory_order_relaxed);
      continue;
    }
    Frame f;
    f.kind = Frame::Kind::Data;
    f.recv = recv;
    f.size = static_cast<std::uint16_t>(size);
    std::memcpy(f.bytes.data(), buf.data(), size);
    if (queue.try_push(f)) {
      stats.enqueued.fetch_add(1, std::memory_order_relaxed);
    } else {
      stats.queue_full.fetch_add(1, std::memory_order_relaxed);
    }
  }
}

}  // namespace sifted::net
