#include "sifted/net/system_clock.hpp"

#include <chrono>

namespace sifted::net {

Timestamp SystemClock::now() const noexcept {
  const std::int64_t t = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
  std::int64_t prev = last_.load(std::memory_order_relaxed);
  while (t > prev && !last_.compare_exchange_weak(prev, t, std::memory_order_relaxed)) {
  }
  return Timestamp{t > prev ? t : prev};
}

}  // namespace sifted::net
