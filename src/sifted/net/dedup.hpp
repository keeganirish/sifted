#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "sifted/core/sensor.hpp"
#include "sifted/core/types.hpp"

namespace sifted::net {

// Duplicate suppression for ingest (threat model B1: replayed or duplicated
// datagrams).
//
// A frame is a duplicate if its (seq, time) key matches one of the last
// `window` keys accepted from the same sensor. Keying on the measurement time
// as well as seq is what tolerates sensor restarts: a restarted sensor starts
// its seq again from 0, but with new timestamps, so its frames are never
// mistaken for repeats; an exact repeat of an earlier datagram has both the
// same seq and the same time. Frames older than the window are not tracked
// here; the engine rejects them as OutOfOrder if they are older than its
// clock.
//
// State is kept only for sensors in the table (indexed like the table), so
// spoofed sensor ids cannot grow memory; frames from unknown sensors pass
// through and are rejected by the engine. Memory: sensors x window keys.
class DuplicateFilter {
 public:
  DuplicateFilter(const SensorTable& sensors, std::size_t window);

  // True if the frame repeats a recently accepted one. Otherwise records it
  // (when the sensor is known) and returns false.
  bool is_duplicate(SensorId sensor, std::uint32_t seq, Timestamp time);

  std::uint64_t suppressed() const noexcept { return suppressed_; }

 private:
  struct Window {
    std::vector<std::pair<std::uint32_t, std::int64_t>> keys;  // ring
    std::size_t head = 0;
  };
  const SensorTable* sensors_;
  std::size_t window_;
  std::vector<Window> per_sensor_;
  std::uint64_t suppressed_ = 0;
};

}  // namespace sifted::net
