#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sifted/core/types.hpp"

// Per-source rate limiting for UDP ingest (threat model B1: flood / resource
// exhaustion; ADR 0010).
//
// A token bucket per source address: each datagram costs one token; a source
// earns `rate_per_s` tokens per second up to `burst`. A datagram from a source
// with less than one token is dropped (and counted) before it is decoded or
// queued, so one noisy or hostile host cannot fill the engine queue and starve
// the others. The table holds at most `max_sources` sources; a new source
// beyond that replaces the one heard from least recently. Memory is
// therefore bounded whatever the traffic -- at the price that a sender who
// can spoof many source addresses gets a fresh bucket per address (UDP source
// addresses are unauthenticated; the queue bound and the queue-full counter
// remain the last line of defence).
//
// Runs on the receive thread, at the live edge, with wall-clock time passed
// in. What it drops never reaches the engine or the event log, so replay is
// unaffected; the drops are counted in UdpStats::rate_limited. Time is a
// parameter, so the class is deterministic and unit-tested without sockets.
namespace sifted::net {

struct RateLimitConfig {
  double rate_per_s = 0.0;  // 0: no limit
  double burst = 0.0;       // bucket size [datagrams], >= 1 when limiting
  std::size_t max_sources = 1024;

  bool enabled() const noexcept { return rate_per_s > 0.0; }
};

bool is_valid(const RateLimitConfig& c) noexcept;

class SourceRateLimiter {
 public:
  // Throws std::invalid_argument if !is_valid(config).
  explicit SourceRateLimiter(RateLimitConfig config);

  // True if a datagram from `source` at `now` is admitted (and charged).
  bool admit(std::uint64_t source, Timestamp now);

  std::size_t sources() const noexcept { return table_.size(); }
  std::uint64_t evictions() const noexcept { return evictions_; }
  const RateLimitConfig& config() const noexcept { return config_; }

 private:
  struct Entry {
    std::uint64_t source;
    double tokens;
    Timestamp last;
  };
  RateLimitConfig config_;
  std::vector<Entry> table_;
  std::uint64_t evictions_ = 0;
};

}  // namespace sifted::net
