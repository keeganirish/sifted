#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "sifted/core/types.hpp"

namespace sifted {

// What a sensor measures. Determines how Measurement::z / sigma are read.
enum class SensorKind : std::uint8_t {
  RangeBearing = 1,  // z = {range [m], bearing [rad]}
  BearingOnly = 2,   // z = {bearing [rad]}
  GeoPosition = 3,   // z = {latitude [deg], longitude [deg], altitude [m]}
};

// Number of leading components of z / sigma that `kind` uses. 0 if unknown.
constexpr std::size_t used_components(SensorKind kind) noexcept {
  switch (kind) {
    case SensorKind::RangeBearing:
      return 2;
    case SensorKind::BearingOnly:
      return 1;
    case SensorKind::GeoPosition:
      return 3;
  }
  return 0;
}

// One sensor observation, in the sensor's own measurement space (no
// coordinate conversion yet; that is the estimator's job).
//
// Bearings are radians in [-pi, pi]. sigma[i] is the 1-sigma noise of z[i],
// in the same units as z[i]. Components beyond used_components(kind) must be
// exactly +0.0 so every measurement has exactly one canonical encoding.
struct Measurement {
  SensorId sensor_id = 0;
  SensorKind kind = SensorKind::BearingOnly;
  std::uint32_t seq = 0;  // per-sensor sequence number, for duplicate detection
  Timestamp time{};       // when the sensor says the observation was made
  std::array<double, 3> z{};
  std::array<double, 3> sigma{};

  friend bool operator==(const Measurement&, const Measurement&) = default;
};

// True iff `m` is something the system is willing to process: known kind,
// finite, in-range values, positive bounded sigmas, canonical unused fields.
// This is the semantic half of the trust boundary; the wire codec handles the
// byte-level half and calls this on everything it decodes.
bool is_valid(const Measurement& m) noexcept;

}  // namespace sifted
