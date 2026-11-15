#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace sifted {

// What happened to one measurement. Every measurement gets exactly one outcome.
enum class MeasurementOutcome : std::uint8_t {
  Initialized,      // started a new (tentative) track
  Updated,          // associated with an existing track and applied
  Invalid,          // failed is_valid() (defence in depth: the decoder checks too)
  UnknownSensor,    // sensor_id not in the sensor table
  KindMismatch,     // measurement kind differs from the sensor's configured kind
  UnsupportedKind,  // cannot be used in this configuration (GeoPosition without an origin)
  Duplicate,        // same sensor, seq and time as that sensor's last accepted measurement
  OutOfOrder,       // older than the engine's current time (dropped and counted)
  FilterRejected,   // associated, but the filter refused the update; track unchanged
  Unassociated,     // gated with no track and its kind cannot start one (bearing-only)
  TrackLimit,       // would have started a track, but the live-track cap is reached
  OutOfFrame,       // geodetic point too far from the tangent-plane origin to project
};
inline constexpr std::size_t kMeasurementOutcomeCount = 12;

std::string_view to_string(MeasurementOutcome o) noexcept;

}  // namespace sifted
