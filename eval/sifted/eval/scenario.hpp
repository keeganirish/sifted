#pragma once

// Shared ground-truth scenario loading and scoring: the one implementation
// behind the regression, degraded-network, fault-injection and consistency
// tests and the sifted_eval scorecard (docs/evaluation.md). Moved out of
// multi_target_regression_test.cpp in M3 and out of tests/ into the
// sifted_eval_support library later, both times unchanged in behaviour, so
// the M2 baselines still hold. Malformed input throws std::runtime_error.
//
// Fixtures: tests/fixtures/scenarios/<name>/{sensors.cfg,frames.hex,truth.csv},
// written by `cd sim && python3 -m sifted_sim.generate <name> <dir>`.
//
// Scoring (CLEAR-MOT style; Bernardin and Stiefelhagen, 2008). At each truth
// time T, once the tracker has consumed its input up to T:
//   * reported tracks are the confirmed and coasting tracks, extrapolated to T;
//   * they are matched one-to-one to the truth targets present at T. As in
//     CLEAR-MOT, a target's previous match is kept while that track is still
//     reported and within kMatchDistanceM (so two targets passing through the
//     same point do not flip the score); the rest are matched by a
//     minimum-total-distance assignment with pairs further apart than
//     kMatchDistanceM forbidden. The matcher knows nothing about which track
//     "should" be which target.
// From the matches:
//   * position RMSE per target, over covered, matched instants at least
//     kRmseBurnInS after the target first enters coverage (start-up excluded);
//   * missed target-instants: covered targets unmatched at least kMissBurnInS
//     after they first enter coverage (time for 3 detections plus
//     confirmation);
//   * ID switches: a target matched to a different track than the last time
//     it was matched;
//   * false tracks: reported tracks matched to any target (covered or not) in
//     fewer than half of the instants they were reported (a track that
//     lingers a few seconds after its target leaves coverage is not false).
// kMatchDistanceM = 100 m: about 5x the radar's raw position error at the
// 8 km edge of coverage (sqrt(10^2 + (8000 * 0.0035)^2) ~ 30 m) once
// filtered, yet below the 120 m lane spacing of close_pass, so a track cannot
// be credited to the wrong lane. It is the default of a per-evaluation
// parameter (Scorer, run_engine, run_pipeline; sifted_eval --match-distance):
// another sensor geometry, such as the real-data evaluation's longer-range
// radar (docs/real-data-evaluation.md), sizes it by the same rule.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/eval/consistency.hpp"
#include "sifted/net/impairment.hpp"
#include "sifted/net/pipeline_config.hpp"

namespace sifted::eval {

inline constexpr double kMatchDistanceM = 100.0;
inline constexpr double kRmseBurnInS = 10.0;
inline constexpr double kMissBurnInS = 5.0;

struct TruthRow {
  std::int64_t time_ns = 0;
  double x = 0.0;
  double y = 0.0;
  int target = 0;
  bool covered = true;  // inside the coverage of a sensor that can start tracks
  double vx = 0.0;
  double vy = 0.0;
};

struct Scenario {
  SensorTable sensors;
  std::vector<Measurement> measurements;  // decoded frames, in send order
  std::vector<TruthRow> truth;            // sorted by time, then target
};

// Loads the scenario in directory `dir` (sensors.cfg, frames.hex, truth.csv).
// Throws std::runtime_error if a file is missing or malformed.
Scenario load_scenario_dir(const std::string& dir);
// Whether `dir` holds the three scenario files.
bool is_scenario_dir(const std::string& dir);
std::vector<std::uint8_t> encode_frame(const Measurement& m);

// What happened at one truth instant: target -> matched track (0: none).
struct InstantRecord {
  std::int64_t time_ns = 0;
  std::map<int, TrackId> match;
  std::map<int, std::int64_t> updated;  // last update time of the matched track
  std::map<int, bool> covered;
};

// Filter consistency (consistency.hpp), over the same matched target-instants
// as the RMSE (covered, matched, past kRmseBurnInS):
//   full      4-dof NEES of [px, py, vx, vy] against truth;
//   position  2-dof NEES of [px, py].
// Split by what the estimate at the truth instant is:
//   updated       the track's last update is at that instant (a posterior);
//   extrapolated  predicted forward from an earlier update (confirmed tracks
//                 between updates, and coasting tracks).
// They test different things: the posterior covariance, and the process noise
// that grows it between updates.
struct NeesStats {
  ChiSquareStat full;
  ChiSquareStat position;
};
struct NeesByRegime {
  NeesStats updated;
  NeesStats extrapolated;
};

struct Score {
  std::map<int, double> rmse;  // per target
  std::map<int, std::size_t> rmse_n;
  std::size_t eligible = 0;  // target-instants past the miss burn-in
  std::size_t missed = 0;
  std::size_t id_switches = 0;
  std::size_t false_tracks = 0;
  std::size_t reported_tracks = 0;  // distinct ids ever reported
  std::uint64_t tracks_started = 0;
  std::uint64_t track_limit = 0;
  std::uint64_t out_of_order = 0;       // measurements the engine dropped as OutOfOrder
  std::uint64_t measurements = 0;       // measurements the engine was offered
  std::size_t bearings = 0;             // bearing-only measurements offered
  std::size_t bearings_associated = 0;  // ... that updated a track
  double mean_rmse = 0.0;               // over targets
  double max_rmse = 0.0;
  std::vector<TrackEstimate> trace;     // every reported estimate, for determinism
  std::vector<InstantRecord> instants;  // per truth instant, for recovery analysis
  NeesByRegime nees;                    // all targets
  std::map<int, NeesByRegime> nees_by_target;
  // NIS (the engine's own gating d^2, read from its trace) of every associated
  // update on a track while it is matched to a covered target past
  // kRmseBurnInS from that target entering coverage, per sensor kind. Updates
  // of unmatched (clutter / false) tracks are left out: their measurements
  // are not of a target the model describes.
  std::map<SensorKind, ChiSquareStat> nis;
  // Associated updates that rolled out of a track's trace window before they
  // could be read (0 unless TraceConfig::events_per_track is too small).
  std::size_t nis_lost = 0;

  double missed_ratio() const {
    return eligible ? static_cast<double>(missed) / static_cast<double>(eligible) : 0.0;
  }
};

// Accumulates a Score over the truth instants of a scenario.
class Scorer {
 public:
  explicit Scorer(const std::vector<TruthRow>& truth, double match_distance_m = kMatchDistanceM);
  // Truth instants, ascending.
  const std::vector<std::int64_t>& instants() const noexcept { return times_; }
  // Scores instant `i` (in order) from the engine's tracks extrapolated to it.
  void score(std::size_t i, const Engine& engine);
  // Final numbers; `engine` supplies the counters.
  Score finish(const Engine& engine);

 private:
  const std::vector<TruthRow>* truth_;
  double match_m_;
  std::vector<std::int64_t> times_;
  std::vector<std::size_t> first_row_;  // per instant
  Score s_;
  std::map<int, std::int64_t> first_covered_;
  std::map<int, TrackId> last_match_;
  std::map<TrackId, std::pair<std::size_t, std::size_t>> track_counts_;  // reported, matched
  std::map<int, double> sq_;
  // Reads the Associated trace events of `track` not read before; adds their
  // NIS if `score` (else only marks them read).
  void collect_nis(const Engine& engine, TrackId track, bool score);
  std::map<TrackId, std::uint64_t> nis_read_;  // Associated events read, per track
  std::map<double, std::size_t> dof_of_gate_;  // gate threshold -> its dof
};

// Feeds the scenario's measurements straight into an Engine, scan by scan
// (as the M2 regression test always did).
Score run_engine(const Scenario& sc, const EngineConfig& cfg = {},
                 double match_distance_m = kMatchDistanceM);

// Same, with the measurement list replaced (fault injection).
Score run_engine(const Scenario& sc, const std::vector<Measurement>& measurements,
                 const EngineConfig& cfg = {}, double match_distance_m = kMatchDistanceM);

// Feeds a sequence of arrivals through a net::Pipeline the way the live
// daemon does: frames in arrival order with recv = arrival time, plus a tick
// whenever the input has been quiet for `tick_ns` while something is pending
// (sifted_fusiond's 100 ms queue timeout), then finish(). Truth instant T is
// scored once every event at or before T has been consumed.
struct PipelineRun {
  Score score;
  std::uint64_t ticks = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t future_rejected = 0;
  std::uint64_t reorder_late = 0;
  std::uint64_t reorder_forced = 0;
  std::size_t held_high_water = 0;
  std::string output;  // the pipeline's deterministic output stream
};
inline constexpr std::int64_t kDaemonTickNs = 100'000'000;
PipelineRun run_pipeline(const Scenario& sc, const std::vector<net::ArrivingFrame>& arrivals,
                         const net::PipelineConfig& cfg, std::int64_t tick_ns = kDaemonTickNs,
                         double match_distance_m = kMatchDistanceM);

// Every frame of the scenario, sent at its measurement time.
std::vector<net::SentFrame> sent_frames(const std::vector<Measurement>& measurements);
// Arrivals with no impairment (arrival = send time, send order).
std::vector<net::ArrivingFrame> clean_arrivals(const std::vector<net::SentFrame>& frames);

// Recovery after a link outage [start, end) (docs/design.md section 9):
//   * the targets considered are those covered and matched at the last truth
//     instant before the outage starts and still covered after it ends;
//   * a target has recovered at truth instant T >= end when it is matched to
//     a track that has been updated since the outage ended (a track that is
//     merely being extrapolated through the gap does not count, however close
//     its prediction happens to be);
//   * recovery_s: from the outage end to the first truth instant at which all
//     of them have recovered (negative: never within the run);
//   * survived: how many are matched, at that instant, to the same track as
//     before the outage (it coasted through); reinitiated: to a new id.
struct Recovery {
  std::size_t targets = 0;
  double recovery_s = -1.0;
  std::size_t survived = 0;
  std::size_t reinitiated = 0;
  std::map<int, bool> target_survived;             // per target considered
  std::map<int, std::int64_t> last_update_before;  // its track's last update before the outage
};

// The Associated trace events of `track` recorded since the engine had
// recorded `read` of them, oldest first; `read` becomes the new total. Events
// that rolled out of the track's trace window before they could be read are
// added to `lost`. Shared by the NIS of Scorer and of the real-data
// evaluation (realdata.hpp), so both read the engine's gating d^2 the same way.
std::vector<store::TraceEvent> new_associated_events(const Engine& engine, TrackId track,
                                                     std::uint64_t& read, std::size_t& lost);
// Degrees of freedom of the chi-square gate an Associated event was compared
// to: the dimension whose gate quantile reproduces `gate` exactly (same
// function, same inputs). Throws std::runtime_error if none does.
std::size_t gate_dof(const Engine& engine, double gate);

// Largest change of a target's true velocity between two times [m/s]
// (0 for a target flying straight and level through the interval).
double velocity_change(const Scenario& sc, int target, std::int64_t from_ns, std::int64_t to_ns);
Recovery recovery_after(const Score& s, std::int64_t outage_start_ns, std::int64_t outage_end_ns);

}  // namespace sifted::eval
