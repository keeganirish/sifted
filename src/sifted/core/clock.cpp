#include "sifted/core/clock.hpp"

namespace sifted {

bool SimulatedClock::advance_to(Timestamp t) noexcept {
  if (t < now_) {
    return false;
  }
  now_ = t;
  return true;
}

}  // namespace sifted
