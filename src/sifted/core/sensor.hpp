#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/core/measurement.hpp"
#include "sifted/core/result.hpp"
#include "sifted/core/types.hpp"

// Static sensor configuration: where each sensor is and which way it points.
// Decision record: docs/adr/0003-sensor-table.md
//
// Frames and angle conventions (used everywhere in the core):
//   * The tracking frame is a local flat x/y plane in metres: +x east, +y north.
//   * Angles are radians, counter-clockwise from the frame's +x axis (the
//     mathematical convention, NOT a compass bearing).
//   * A sensor's yaw is the direction of its boresight in the tracking frame.
//   * A measured bearing is relative to the boresight:
//       bearing = wrap(atan2(target.y - sensor.y, target.x - sensor.x) - yaw)
namespace sifted {

struct SensorPose {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;  // in [-pi, pi]

  friend bool operator==(const SensorPose&, const SensorPose&) = default;
};

struct SensorConfig {
  SensorId id = 0;
  SensorKind kind = SensorKind::RangeBearing;
  SensorPose pose{};

  friend bool operator==(const SensorConfig&, const SensorConfig&) = default;
};

enum class SensorTableError : std::uint8_t {
  InvalidSensor,  // unknown kind, non-finite or out-of-range pose
  DuplicateId,
  TooManySensors,
  // Text format only:
  TooLarge,
  MissingHeader,
  UnsupportedVersion,
  Malformed,  // wrong token count, bad number, unknown keyword or kind name
};

std::string_view to_string(SensorTableError e) noexcept;

// Immutable-after-setup lookup from SensorId to configuration. Entries are kept
// sorted by id, so iteration order (and the canonical text form) depends only
// on the contents, never on insertion order or addresses.
class SensorTable {
 public:
  static constexpr std::size_t kMaxSensors = 1024;
  // |x|, |y| bound for a pose: generous for a local tangent plane.
  static constexpr double kMaxOffsetM = 1.0e7;

  // Validates and inserts. On error the table is unchanged.
  std::optional<SensorTableError> add(const SensorConfig& cfg);

  const SensorConfig* find(SensorId id) const noexcept;
  // Position of `id` in iteration order; lets callers keep per-sensor state in
  // a plain vector sized to size() without any allocation per measurement.
  std::optional<std::size_t> index_of(SensorId id) const noexcept;

  std::size_t size() const noexcept { return sensors_.size(); }
  bool empty() const noexcept { return sensors_.empty(); }
  const std::vector<SensorConfig>& sensors() const noexcept { return sensors_; }

  friend bool operator==(const SensorTable&, const SensorTable&) = default;

 private:
  std::vector<SensorConfig> sensors_;  // sorted by id, ids unique
};

bool is_valid(const SensorConfig& cfg) noexcept;

// ---- Text form ("sifted-sensors 1") ---------------------------------------
//
//   # comments and blank lines are ignored
//   sifted-sensors 1
//   sensor <id> <kind> <x_m> <y_m> <yaw_rad>
//
// <kind> is range_bearing | bearing_only | geo_position. Numbers are decimal
// (doubles are parsed with std::from_chars, so the shortest round-trip form a
// Python repr() writes is read back bit-exactly). Tokens are separated by
// spaces or tabs. The file is operator configuration, not network input, but it
// is still parsed strictly and with size limits.
inline constexpr std::size_t kMaxSensorTableTextSize = 64 * 1024;

Result<SensorTable, SensorTableError> parse_sensor_table(std::string_view text);

// Canonical text form: header plus one line per sensor in id order, numbers in
// shortest round-trip form. parse_sensor_table(format_sensor_table(t)) == t.
// This exact text is what a recording will carry so replay rebuilds the table.
std::string format_sensor_table(const SensorTable& table);

}  // namespace sifted
