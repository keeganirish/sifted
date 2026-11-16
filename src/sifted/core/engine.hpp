#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "sifted/association/assignment.hpp"
#include "sifted/association/lifecycle.hpp"
#include "sifted/core/measurement.hpp"
#include "sifted/core/outcome.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/core/types.hpp"
#include "sifted/estimation/cv_filter.hpp"
#include "sifted/estimation/geodetic.hpp"
#include "sifted/store/track_store.hpp"

namespace sifted {

// The fusion core: association -> estimation -> track store.
//
// Contract (what makes deterministic replay possible):
//   * Single-threaded. Concurrency lives around the engine (network threads
//     feed an ordered queue); never inside it.
//   * Its output is a pure function of the sensor table and configuration it
//     was built with and the ordered sequence of inputs it is given. It reads
//     no wall clock, no RNG seeded from the machine, and iterates no container
//     whose order depends on addresses or hashes. Ties are broken by track id.
//
// M2: many tracks, many sensors. Each group of measurements that share a
// sensor and a timestamp (one radar scan, or a single measurement) is
// processed jointly:
//   1. screen: validity, sensor table lookup, kind, duplicates, ordering;
//   2. advance the engine clock to the group time and run the track lifecycle
//      (coast / drop) at that time;
//   3. gate every measurement against every live track with a chi-square gate
//      on the Mahalanobis distance in that sensor's measurement space;
//   4. assign with global nearest neighbour (Hungarian, per connected cluster)
//      in two passes: confirmed and coasting tracks first, then tentative
//      tracks with what is left, so a young track cannot steal a measurement
//      from an established one;
//   5. update assigned tracks; start tentative tracks from the remaining
//      measurements that can initiate one (not bearing-only), up to a cap.
// Design and parameter reasoning: docs/design.md sections 6-7, ADR 0004.

struct AssociationConfig {
  // Probability that the gate keeps a correctly associated measurement. 0.999
  // makes a lost true association (and the track break it causes) a 1-in-1000
  // event per update; the wider gate costs little because assignment is
  // global rather than greedy. Gate thresholds: 10.83 (1-D), 13.82 (2-D).
  double gate_probability = 0.999;
  // Measurements with gate < d^2 <= near_miss_factor * gate are recorded in a
  // track's trace as near misses (no effect on association).
  double near_miss_factor = 4.0;
  // Global nearest neighbour (the default); Greedy only for comparisons.
  association::AssignmentMethod method = association::AssignmentMethod::GlobalNearestNeighbour;
  // Skip the full gating computation for pairs that provably cannot be
  // within near_miss_factor x gate (estimation::may_gate). Performance only:
  // results are identical either way, so it is not recorded in event logs;
  // false exists so tests and the engine fuzzer can prove that.
  bool gate_screen = true;

  friend bool operator==(const AssociationConfig&, const AssociationConfig&) = default;
};

// Manoeuvre handling by process-noise switching (Bar-Shalom, Li and
// Kirubarajan, "Estimation with Applications to Tracking and Navigation",
// 2001, sec. 11.2-11.3). A constant-velocity filter sized for straight flight
// lags in a turn; its innovations then grow and stay large, and a gate on them
// would eventually lose the target. Each track keeps a fading-memory sum of
// the normalised innovation squared of its position-observing updates,
//   rho_k = alpha * rho_{k-1} + d2_k,
// which for a consistent filter has mean n/(1-alpha) and is approximately
// chi-square with n(1+alpha)/(1-alpha) degrees of freedom scaled by
// 1/(1+alpha) (n = 2). When rho exceeds that distribution's
// `detect_probability` quantile the track's process noise is multiplied by
// `accel_psd_factor`; when rho falls back below its consistent mean, the
// nominal value returns. Both transitions are traced.
struct ManoeuvreConfig {
  // alpha: effective memory 1/(1-alpha) = 5 updates, short enough to react
  // within a few seconds of a turn starting.
  double fading = 0.8;
  // False-detection probability per update on a consistent filter is 1%.
  double detect_probability = 0.99;
  // q x 10 ~ sqrt(10) x the acceleration: covers ~6 m/s^2 turns with the
  // default q. 1.0 disables manoeuvre handling.
  double accel_psd_factor = 10.0;

  friend bool operator==(const ManoeuvreConfig&, const ManoeuvreConfig&) = default;
};

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
  AssociationConfig association;
  ManoeuvreConfig manoeuvre;
  association::LifecycleConfig lifecycle;
  // Hard cap on live tracks (threat model: resource exhaustion). Measurements
  // that would start a track beyond it are counted as TrackLimit.
  std::size_t max_tracks = 1024;
  // Tangent-plane origin for GeoPosition measurements. Without one, they are
  // rejected as UnsupportedKind. Fixed for a run (ADR 0007).
  std::optional<estimation::GeoOrigin> origin;
  store::TraceConfig trace;

  friend bool operator==(const EngineConfig&, const EngineConfig&) = default;
};

// True iff every parameter is finite and in range.
bool is_valid(const EngineConfig& c) noexcept;

struct MeasurementResult {
  MeasurementOutcome outcome = MeasurementOutcome::Invalid;
  TrackId track = 0;  // track updated or started; 0 otherwise

  friend bool operator==(const MeasurementResult&, const MeasurementResult&) = default;
};

// Snapshot of one track.
struct TrackEstimate {
  TrackId id = 0;
  association::TrackStatus status = association::TrackStatus::Tentative;
  Timestamp time{};         // time the estimate refers to
  Timestamp last_update{};  // last applied measurement
  Timestamp created{};
  estimation::StateVector state;  // [px, py, vx, vy] in the tracking frame
  estimation::StateCovariance covariance;
  std::uint64_t updates = 0;  // measurements applied, including the initialising one
  SensorId last_sensor = 0;
  std::vector<SensorId> sensors;  // contributing sensors, ascending
  bool manoeuvring = false;       // raised process noise in effect
};

class Engine {
 public:
  // An engine with no sensors: it validates and counts input but rejects every
  // measurement as UnknownSensor.
  Engine() : Engine(SensorTable{}) {}
  // Throws std::invalid_argument if !is_valid(config).
  explicit Engine(SensorTable sensors, EngineConfig config = {});

  // Processes one measurement on its own (a group of one).
  MeasurementOutcome on_measurement(const Measurement& m);
  MeasurementResult on_measurement_result(const Measurement& m);

  // Processes measurements in order. Each maximal run of consecutive
  // measurements with the same sensor and timestamp is associated jointly
  // (global nearest neighbour). Appends exactly one result per measurement to
  // `out`, in input order.
  void on_scan(std::span<const Measurement> scan, std::vector<MeasurementResult>& out);

  // Live tracks (any status but Dropped), sorted by id, at their own last
  // update time.
  std::vector<TrackEstimate> tracks() const;
  std::optional<TrackEstimate> track(TrackId id) const;
  // Estimates extrapolated to `t` (>= each track's time) without changing
  // state. Tracks that cannot be predicted to `t` are omitted.
  std::vector<TrackEstimate> tracks_at(Timestamp t) const;
  std::optional<TrackEstimate> track_at(TrackId id, Timestamp t) const;

  // The engine clock: time of the latest accepted measurement group.
  std::optional<Timestamp> time() const noexcept { return now_; }

  // Every measurement offered, whatever its outcome.
  std::uint64_t measurements_processed() const noexcept { return processed_; }
  std::uint64_t count(MeasurementOutcome o) const noexcept {
    return outcome_counts_[static_cast<std::size_t>(o)];
  }
  std::uint64_t tracks_started() const noexcept { return next_track_id_ - 1; }
  std::uint64_t tracks_dropped(association::DropReason r) const noexcept {
    return drop_counts_[static_cast<std::size_t>(r)];
  }

  // Gate threshold on d^2 for a measurement of dimension `dim` (1 or 2).
  double gate(std::size_t dim) const noexcept { return gates_[dim < 2 ? 0 : 1]; }
  // Manoeuvre detector thresholds on the fading-memory NIS sum.
  double manoeuvre_on_threshold() const noexcept { return manoeuvre_on_; }
  double manoeuvre_off_threshold() const noexcept { return manoeuvre_off_; }

  const store::TrackStore& store() const noexcept { return store_; }
  // "Why does the system believe this track?" See TrackStore::explain.
  std::optional<std::string> explain(TrackId id) const { return store_.explain(id); }

  const SensorTable& sensors() const noexcept { return sensors_; }
  const EngineConfig& config() const noexcept { return config_; }

 private:
  void process_group(std::span<const Measurement> group, std::vector<MeasurementResult>& out);
  void maintain(Timestamp t);
  void update_manoeuvre(store::TrackRecord& r, double d2, Timestamp t);
  TrackEstimate estimate_of(const store::TrackRecord& r) const;

  SensorTable sensors_;
  EngineConfig config_;
  std::array<double, 2> gates_{};
  double manoeuvre_on_ = 0.0;
  double manoeuvre_off_ = 0.0;
  store::TrackStore store_;
  std::optional<Timestamp> now_;
  TrackId next_track_id_ = 1;

  // Last accepted (seq, time) per sensor, indexed like sensors_.sensors().
  std::vector<std::optional<std::pair<std::uint32_t, std::int64_t>>> last_seq_;

  std::uint64_t processed_ = 0;
  std::array<std::uint64_t, kMeasurementOutcomeCount> outcome_counts_{};
  std::array<std::uint64_t, 4> drop_counts_{};
};

}  // namespace sifted
