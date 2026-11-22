#pragma once

// Track report: a tracker-independent interchange format for reported tracks,
// so any tracker (Sifted, or an external baseline such as Stone Soup,
// benchmarks/baselines/stonesoup/) is scored by the one Scorer
// (scenario.hpp). ADR 0015.
//
// Text, ASCII, LF (CR LF tolerated), one header line, one column line, then
// one row per (truth instant, track):
//
//   sifted-tracks 1
//   time_ns,track_id,status,last_update_ns,x_m,y_m,vx_mps,vy_mps,p00,p01,p02,p03,p11,p12,p13,p22,p23,p33
//   5000000000,1,confirmed,5000000000,1203.5,-88.25,70.1,0.4,104.2,...
//
//   time_ns         the truth instant the estimate refers to (integer ns,
//                   the scenario's truth.csv clock); the tracker has consumed
//                   every measurement stamped at or before it
//   track_id        integer in [1, 2^63); identity across instants
//   status          confirmed | coasting | tentative. Only confirmed and
//                   coasting tracks count as reported (as for Sifted)
//   last_update_ns  time of the track's last applied measurement, <= time_ns
//   x_m .. vy_mps   state [px, py, vx, vy] in the scenario's tracking frame
//   p00 .. p33      upper triangle of the 4x4 state covariance, row by row,
//                   in the same state order; either all ten present (then
//                   symmetric positive definite) or all ten empty, the same
//                   for every row of a file. Without them NEES is unavailable.
//
// Rows are sorted by (time_ns, track_id), strictly: a duplicate or an
// unsorted row is an error. A truth instant with no row is an empty report
// (the tracker reported nothing), not an error.
//
// Untrusted input (docs/threat-model.md, B7): parsed strictly, with size,
// line-length and row-count limits checked before anything is allocated;
// every number must be finite and within physical bounds; fuzzed
// (fuzz/fuzz_track_report.cpp). Errors throw TrackReportError naming the line.
// Scoring adds the checks that need the truth: every time_ns must be one of
// the scenario's truth instants (so none outside its range), and a track id
// that was missing at some truth instant after being reported may not appear
// again (an id reused for another object would fake continuity).

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/eval/gospa.hpp"
#include "sifted/eval/scenario.hpp"

namespace sifted::eval {

inline constexpr std::string_view kTrackReportHeader = "sifted-tracks 1";
inline constexpr std::string_view kTrackReportColumns =
    "time_ns,track_id,status,last_update_ns,x_m,y_m,vx_mps,vy_mps,p00,p01,p02,p03,p11,p12,p13,p22,"
    "p23,p33";
inline constexpr std::size_t kMaxTrackReportBytes = std::size_t{256} << 20;
inline constexpr std::size_t kMaxTrackReportLine = 1024;
inline constexpr std::size_t kMaxTrackReportRows = 10'000'000;
// Bounds: positions within 10^8 m (beyond any sensor's tracking frame),
// speeds within 10^5 m/s, covariance entries within 10^16.
inline constexpr double kMaxTrackPositionM = 1.0e8;
inline constexpr double kMaxTrackSpeedMps = 1.0e5;
inline constexpr double kMaxTrackCovariance = 1.0e16;

class TrackReportError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct ReportedTrack {
  std::int64_t time_ns = 0;
  TrackId id = 0;
  association::TrackStatus status = association::TrackStatus::Confirmed;
  std::int64_t last_update_ns = 0;
  estimation::StateVector state = estimation::StateVector::Zero();
  estimation::StateCovariance covariance = estimation::StateCovariance::Identity();

  friend bool operator==(const ReportedTrack&, const ReportedTrack&) = default;
};

struct TrackReport {
  bool covariance = false;  // rows carry covariances
  std::vector<ReportedTrack> rows;

  friend bool operator==(const TrackReport&, const TrackReport&) = default;
};

// Throws TrackReportError on any violation of the format above.
TrackReport parse_track_report(std::string_view text);
// Canonical text: shortest round-trip number form, LF line ends. parse() of
// it gives back the same report. Throws TrackReportError if `report` is not
// itself valid (unsorted, out of bounds, ...).
std::string format_track_report(const TrackReport& report);
// Reads and parses a file; the size limit is checked before reading.
TrackReport read_track_report_file(const std::string& path);

// Appends every live track the engine holds at truth instant `t_ns`
// (tracks_at, any status), sorted by id, with covariance: what
// run_engine's scorer saw, as a track report (InstantObserver).
void append_engine_tracks(TrackReport& report, std::int64_t t_ns, const Engine& engine);

// Scores a track report against the scenario truth with the shared Scorer.
// Score::engine_metrics is false; Score::covariance is report.covariance.
// Throws TrackReportError for a row whose time is not a truth instant or a
// reused track id.
Score score_track_report(const std::vector<TruthRow>& truth, const TrackReport& report,
                         double match_distance_m = kMatchDistanceM, const GospaParams& gospa = {});

}  // namespace sifted::eval
