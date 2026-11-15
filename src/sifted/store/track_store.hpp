#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/association/lifecycle.hpp"
#include "sifted/core/outcome.hpp"
#include "sifted/core/types.hpp"
#include "sifted/estimation/cv_filter.hpp"
#include "sifted/store/ring.hpp"

// The track table and the decision trace behind every track.
//
// For each track the store keeps how it was born, when it was confirmed, and a
// bounded window of the most recent decisions about it: measurements associated
// (with their gating distance), measurements that fell inside its gate but were
// assigned elsewhere, near misses just outside the gate, updates the filter
// refused, and lifecycle transitions. Per-kind totals are kept even after old
// events roll out of the window. Dropped tracks move to a bounded history so
// "why did the system believe track N?" can still be answered after it is gone.
// A global bounded log keeps the latest per-measurement outcomes, including
// rejected ones.
//
// Everything is bounded (TraceConfig) and ordered by insertion or by track id,
// so the trace is as deterministic as the engine that writes it.
namespace sifted::store {

enum class TraceEventKind : std::uint8_t {
  Initiated,       // track started from this measurement
  Associated,      // measurement assigned to this track and applied
  Competed,        // inside this track's gate, but assigned elsewhere / left out
  NearMiss,        // outside the gate, within near_miss_factor x gate
  UpdateRejected,  // assigned, but the filter refused the update
  Confirmed,
  Coasting,
  Recovered,  // coasting -> confirmed
  Dropped,
  ManoeuvreStart,  // innovation statistic exceeded its threshold: process noise raised
  ManoeuvreEnd,    // innovations consistent again: nominal process noise restored
};
inline constexpr std::size_t kTraceEventKindCount = 11;

std::string_view to_string(TraceEventKind k) noexcept;

struct TraceEvent {
  TraceEventKind kind = TraceEventKind::Initiated;
  Timestamp time{};
  SensorId sensor = 0;  // measurement events only
  std::uint32_t seq = 0;
  // Gating statistic and the gate it was compared to (measurement events), or
  // the fading-memory innovation statistic and its threshold (manoeuvre events).
  double distance2 = std::numeric_limits<double>::quiet_NaN();
  double gate = std::numeric_limits<double>::quiet_NaN();
  TrackId other = 0;  // Competed: the track that got the measurement (0: none)
  association::DropReason drop_reason = association::DropReason::None;
};

// One line of text, e.g. "t=1000000000 associated sensor 1 seq 4 d2=1.2 gate=13.8".
std::string format(const TraceEvent& e);

struct TraceConfig {
  std::size_t events_per_track = 64;  // recent-decision window per track
  std::size_t dropped_tracks_kept = 64;
  std::size_t measurement_log = 1024;  // latest per-measurement outcomes
  std::size_t sensors_per_track = 32;  // contributing sensors listed per track

  friend bool operator==(const TraceConfig&, const TraceConfig&) = default;
};

struct SensorContribution {
  SensorId sensor = 0;
  std::uint64_t updates = 0;
  Timestamp last{};
};

struct TrackRecord {
  TrackRecord(TrackId track_id, const estimation::ConstantVelocityFilter& f, Timestamp t,
              const TraceConfig& cfg)
      : id(track_id),
        filter(f),
        created(t),
        time(t),
        last_position_time(t),
        recent(cfg.events_per_track) {}

  TrackId id;
  association::TrackStatus status = association::TrackStatus::Tentative;
  estimation::ConstantVelocityFilter filter;
  Timestamp created;
  Timestamp time;  // time of the last applied measurement (the filter's time)
  // Time of the last position-observing update. The lifecycle timers run from
  // here: bearings refine a track but cannot hold it (ADR 0004).
  Timestamp last_position_time;
  std::optional<Timestamp> confirmed_at;
  std::uint64_t updates = 0;        // all applied measurements, including the first
  std::uint32_t position_hits = 0;  // updates from position-observing sensors
  SensorId last_sensor = 0;
  std::vector<SensorContribution> sensors;  // sorted by sensor id, bounded
  std::uint64_t sensors_omitted = 0;        // updates from sensors beyond the bound
  association::DropReason drop_reason = association::DropReason::None;
  Timestamp dropped_at{};
  // Manoeuvre detector: fading-memory sum of position-observing NIS values.
  double nis_average = 0.0;
  bool manoeuvring = false;

  // Decision trace.
  std::optional<TraceEvent> initiated;
  std::optional<TraceEvent> confirmed;
  Ring<TraceEvent> recent;
  std::array<std::uint64_t, kTraceEventKindCount> totals{};
};

struct MeasurementRecord {
  Timestamp time{};
  SensorId sensor = 0;
  std::uint32_t seq = 0;
  MeasurementOutcome outcome = MeasurementOutcome::Invalid;
  TrackId track = 0;  // track updated or started, 0 if none
};

class TrackStore {
 public:
  explicit TrackStore(TraceConfig cfg = {});

  const TraceConfig& config() const noexcept { return cfg_; }

  // Live tracks, sorted by id.
  const std::vector<TrackRecord>& live() const noexcept { return live_; }
  std::vector<TrackRecord>& live() noexcept { return live_; }
  std::size_t size() const noexcept { return live_.size(); }

  TrackRecord* find(TrackId id) noexcept;
  const TrackRecord* find(TrackId id) const noexcept;
  // Live or recently dropped.
  const TrackRecord* find_any(TrackId id) const noexcept;
  const std::deque<TrackRecord>& dropped() const noexcept { return dropped_; }

  // Adds a new live track. Precondition: its id is larger than every id ever
  // added (ids are never reused), which keeps live() sorted by appending.
  TrackRecord& add(TrackRecord record);

  // Moves the live track `id` to the dropped history (oldest history entries
  // are discarded beyond dropped_tracks_kept). Precondition: find(id) != nullptr.
  void drop(TrackId id, Timestamp t, association::DropReason reason);

  // Appends a decision to `track`'s trace (and its totals).
  void record(TrackRecord& track, const TraceEvent& e);
  // Notes that `sensor` contributed an update at time t.
  void credit(TrackRecord& track, SensorId sensor, Timestamp t);

  void record_measurement(const MeasurementRecord& r) { measurements_.push(r); }
  const Ring<MeasurementRecord>& measurements() const noexcept { return measurements_; }

  // "Why does the system believe this track?" A human-readable account of the
  // track's birth, confirmation, contributing sensors, recent decisions and
  // totals. nullopt if the id is neither live nor in the dropped history.
  std::optional<std::string> explain(TrackId id) const;

 private:
  TraceConfig cfg_;
  std::vector<TrackRecord> live_;  // sorted by id
  std::deque<TrackRecord> dropped_;
  Ring<MeasurementRecord> measurements_;
};

}  // namespace sifted::store
