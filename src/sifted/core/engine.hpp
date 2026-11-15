#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

#include "sifted/core/measurement.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/core/types.hpp"
#include "sifted/estimation/cv_filter.hpp"

namespace sifted {

// The fusion core: association -> estimation -> track store.
//
// Contract (what makes deterministic replay possible):
//   * Single-threaded. Concurrency lives around the engine (network threads
//     feed an ordered queue); never inside it.
//   * Its output is a pure function of the sensor table it was built with and
//     the ordered sequence of inputs it is given. It reads no wall clock, no
//     RNG seeded from the machine, and iterates no container whose order
//     depends on addresses or hashes.
//
// M1 scope: one track, fed by range-bearing sensors. The first accepted
// measurement initialises the track; every later one is predicted to and
// applied with the EKF. There is no association yet: every measurement is
// assumed to belong to the single track. Multi-target association, track
// lifecycle (coast/drop) and other sensor kinds arrive in M2.

struct EngineConfig {
  // White-noise acceleration PSD for the constant-velocity model [m^2/s^3].
  // Sized with the usual rule of thumb q ~ (0.5..1) * a_max^2 * T for
  // manoeuvres up to a_max ~ 2-3 m/s^2 (e.g. a standard-rate 3 deg/s turn at
  // ~60 m/s) at a nominal T = 1 s update interval. Lower q smooths straight
  // flight better but lags in turns; it is a model parameter, not fitted to
  // any one scenario (docs/design.md section 7 records the trade-off).
  double accel_psd = 4.0;
  // 1-sigma prior on each velocity component when a track is started from a
  // single position fix [m/s]. Covers anything up to roughly airliner speed.
  double initial_velocity_sigma = 100.0;
};

// What happened to one measurement. Every measurement gets exactly one outcome.
enum class MeasurementOutcome : std::uint8_t {
  Initialized,      // started the track
  Updated,          // predicted to its time and applied
  Invalid,          // failed is_valid() (defence in depth: the decoder checks too)
  UnknownSensor,    // sensor_id not in the sensor table
  KindMismatch,     // measurement kind differs from the sensor's configured kind
  UnsupportedKind,  // not usable by the M1 tracker (bearing-only, geo-position)
  Duplicate,        // same sensor and seq as that sensor's last accepted measurement
  OutOfOrder,       // older than the track's current time (dropped and counted)
  FilterRejected,   // the filter refused it (e.g. degenerate geometry); state unchanged
};
inline constexpr std::size_t kMeasurementOutcomeCount = 9;

std::string_view to_string(MeasurementOutcome o) noexcept;

// Snapshot of the single M1 track.
struct TrackEstimate {
  TrackId id = 0;
  Timestamp time{};               // time the estimate refers to (last applied measurement)
  estimation::StateVector state;  // [px, py, vx, vy] in the tracking frame
  estimation::StateCovariance covariance;
  std::uint64_t updates = 0;  // measurements applied, including the initialising one
  SensorId last_sensor = 0;
};

class Engine {
 public:
  // An engine with no sensors: it validates and counts input but rejects every
  // measurement as UnknownSensor.
  Engine() : Engine(SensorTable{}) {}
  explicit Engine(SensorTable sensors, EngineConfig config = {});

  MeasurementOutcome on_measurement(const Measurement& m) noexcept;

  // The current track, if one has been started.
  std::optional<TrackEstimate> track() const;
  // Current estimate extrapolated to `t` (>= track time) without changing state.
  std::optional<TrackEstimate> track_at(Timestamp t) const;

  // Every measurement offered, whatever its outcome.
  std::uint64_t measurements_processed() const noexcept { return processed_; }
  std::uint64_t count(MeasurementOutcome o) const noexcept {
    return outcome_counts_[static_cast<std::size_t>(o)];
  }

  const SensorTable& sensors() const noexcept { return sensors_; }
  const EngineConfig& config() const noexcept { return config_; }

 private:
  MeasurementOutcome process(const Measurement& m) noexcept;
  MeasurementOutcome initialize(const Measurement& m, const SensorConfig& sensor) noexcept;

  SensorTable sensors_;
  EngineConfig config_;

  struct Track {
    TrackId id;
    estimation::ConstantVelocityFilter filter;
    Timestamp time;
    std::uint64_t updates;
    SensorId last_sensor;
  };
  std::optional<Track> track_;
  TrackId next_track_id_ = 1;

  // Last accepted seq per sensor, indexed like sensors_.sensors().
  std::vector<std::optional<std::uint32_t>> last_seq_;

  std::uint64_t processed_ = 0;
  std::uint64_t outcome_counts_[kMeasurementOutcomeCount] = {};
};

}  // namespace sifted
