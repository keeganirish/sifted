#pragma once

// Real-data evaluation A: real ADS-B position reports through the real
// decoder and engine, scored by withheld reports (docs/real-data-evaluation.md).
//
// Real reports have no independent truth, so the score is predictive: a
// deterministic subset of each aircraft's reports is withheld from the
// tracker, and at each withheld report's time the tracker's estimate is
// compared with the withheld position. The aircraft address (icao24) never
// reaches the tracker: the input frames carry only time, position and
// sensor, and the run directory holds the address only as a small integer
// label in files the engine never reads (feed_key.csv, withheld.csv), which
// the scorer uses to know which track to ask and whose report it was.
//
// Run directory (written by sim/sifted_sim/realeval.py):
//   sensors.cfg   sensor table (one geo_position sensor)
//   origin.txt    "lat lon alt" of the tangent-plane origin (ADR 0007)
//   frames.hex    tracker input: one wire-v1 GeoPosition frame per line, in
//                 time order
//   feed_key.csv  frame,aircraft,speed_mps,track_deg: per frame, its
//                 aircraft label and the velocity reported with it (empty if
//                 none). Scoring only.
//   withheld.csv  time_ns,aircraft,lat,lon,alt_m: the withheld reports, in
//                 time order. Scoring only.
// The files are produced by our own tool from validated reports; like the
// scenario fixtures they are parsed strictly and any defect throws
// std::runtime_error. The untrusted boundary is the CSV reader in
// sim/sifted_sim/statevectors.py (threat model B6).
//
// At each withheld report (aircraft a, time T), after the engine has consumed
// every frame stamped at or before T:
//   * horizon: T minus the time of a's last fed report;
//   * tracker: the operator picture at T (Engine::tracks_if_silent_until),
//     the track that a's last fed report updated or started, if it is
//     confirmed or coasting there; otherwise counted as "no track";
//   * hold: a's last fed report itself (zero-order hold, raw ADS-B);
//   * dead reckoning: that report moved on by its reported ground velocity.
// Errors are horizontal distances in the run's tangent plane.
//
// NIS: the engine's gating d^2 of every associated update after a track's
// first kNisBurnInUpdates (2-dof for geodetic positions), read from its trace
// with the same helper as the simulator scorecard.
//
// Continuity, over fed reports, per aircraft label:
//   * a track is "confirmed" once it is Confirmed after one of its updates;
//   * fragments: confirmed tracks an aircraft's reports went to, minus one;
//   * never confirmed: aircraft none of whose reports reached a confirmed track;
//   * coverage: of an aircraft's fed reports after its first confirm_hits in
//     the run, the share that updated a track confirmed at that point;
//   * swaps: consecutive updates of one confirmed track from different
//     aircraft; mixed tracks: confirmed tracks updated by more than one
//     aircraft; inconsistent tracks: confirmed tracks on which no aircraft
//     has more than half of the updates.

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/core/measurement.hpp"
#include "sifted/core/outcome.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/estimation/geodetic.hpp"
#include "sifted/eval/consistency.hpp"

namespace sifted::eval {

inline constexpr std::size_t kNisBurnInUpdates = 10;

struct FeedKey {
  int aircraft = 0;
  std::optional<double> speed_mps;  // reported ground speed
  std::optional<double> track_deg;  // reported true track, clockwise from north
};

struct WithheldReport {
  std::int64_t time_ns = 0;
  int aircraft = 0;
  estimation::Geodetic position;
};

struct HoldoutRun {
  SensorTable sensors;
  estimation::GeoOrigin origin;
  std::vector<Measurement> measurements;  // tracker input, decoded by the real decoder
  std::vector<FeedKey> key;               // one per measurement
  std::vector<WithheldReport> withheld;   // ascending time
};

bool is_holdout_dir(const std::string& dir);
HoldoutRun load_holdout_dir(const std::string& dir);

enum class Estimator : std::uint8_t { Tracker, Hold, DeadReckoning };
inline constexpr std::array<Estimator, 3> kEstimators{Estimator::Tracker, Estimator::Hold,
                                                      Estimator::DeadReckoning};
const char* to_string(Estimator e) noexcept;

struct ErrorSample {
  double horizon_s = 0.0;
  double error_m = 0.0;
};

struct AircraftContinuity {
  std::size_t fed = 0;
  std::size_t confirmed_tracks = 0;
  std::size_t coverage_eligible = 0;
  std::size_t covered = 0;
};

struct HoldoutScore {
  std::array<std::vector<ErrorSample>, kEstimators.size()> errors;  // by Estimator
  std::size_t withheld = 0;
  std::size_t no_prior_report = 0;  // withheld before the aircraft's first fed report
  std::size_t no_track = 0;         // the tracker had no reported track to offer
  std::size_t no_velocity = 0;      // dead reckoning impossible (no reported velocity)
  ChiSquareStat nis;
  std::size_t nis_lost = 0;
  std::uint64_t tracks_started = 0;
  std::size_t confirmed_tracks = 0;
  std::size_t fragments = 0;
  std::size_t never_confirmed = 0;
  std::size_t swaps = 0;
  std::size_t mixed_tracks = 0;
  std::size_t inconsistent_tracks = 0;
  std::size_t max_range_m = 0;  // furthest fed report from the origin (rounded down)
  std::map<MeasurementOutcome, std::uint64_t> outcomes;
  std::map<int, AircraftContinuity> aircraft;

  void merge(const HoldoutScore& o);
  std::size_t coverage_eligible() const;
  std::size_t covered() const;
};

// Runs the engine (configured as `cfg`, with the run's origin) over the run.
HoldoutScore score_holdout(const HoldoutRun& run, EngineConfig cfg);

struct Quantiles {
  std::size_t n = 0;
  double median = 0.0;
  double p95 = 0.0;
  double p99 = 0.0;
  double mean = 0.0;
};
// Nearest-rank quantiles (the smallest value with at least p of the sample
// at or below it); all zero for an empty sample.
Quantiles quantiles(std::vector<double> v);

}  // namespace sifted::eval
