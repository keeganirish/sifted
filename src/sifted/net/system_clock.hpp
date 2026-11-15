#pragma once

#include <atomic>
#include <cstdint>

#include "sifted/core/clock.hpp"

namespace sifted::net {

// The machine's wall clock, for the live application edge ONLY (receive time
// stamps in the daemon). Core code never uses it: it takes time as input
// (docs/design.md section 3). Whatever this returns is recorded in the event
// log, so a replay sees the same values without reading any clock.
//
// Never goes backwards, even if the system clock is stepped: each reading is
// clamped to be >= the previous one (lock-free; safe from several threads).
class SystemClock final : public Clock {
 public:
  Timestamp now() const noexcept override;

 private:
  mutable std::atomic<std::int64_t> last_{INT64_MIN};
};

}  // namespace sifted::net
