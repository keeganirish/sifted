#pragma once

#include <compare>
#include <cstdint>

namespace sifted {

using SensorId = std::uint16_t;
using TrackId = std::uint64_t;

// A point in time on the system's clock, as integer nanoseconds.
//
// Time is an *input* to the system (see Clock). It is an integer so that
// arithmetic and comparison are exact and identical on every platform, which
// deterministic replay depends on.
struct Timestamp {
  std::int64_t ns = 0;

  friend constexpr auto operator<=>(const Timestamp&, const Timestamp&) = default;
};

}  // namespace sifted
