#pragma once

#include "sifted/core/types.hpp"

namespace sifted {

// Source of "now". Core code takes a Clock (or, better, a Timestamp) and never
// reads the machine's clock directly. That single rule is what makes record
// and replay deterministic: replay drives a SimulatedClock from the log.
class Clock {
 public:
  virtual ~Clock() = default;
  virtual Timestamp now() const noexcept = 0;
};

// A clock that only moves when told to, and never backwards.
class SimulatedClock final : public Clock {
 public:
  explicit SimulatedClock(Timestamp start = {}) noexcept : now_(start) {}

  Timestamp now() const noexcept override { return now_; }

  // Advances to `t`. Returns false and leaves the clock unchanged if `t` is in
  // the past; the caller decides whether that is a sensor fault or a bug.
  bool advance_to(Timestamp t) noexcept;

 private:
  Timestamp now_;
};

}  // namespace sifted
