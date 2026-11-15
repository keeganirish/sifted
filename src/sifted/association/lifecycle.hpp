#pragma once

#include <cstdint>
#include <string_view>

// Track lifecycle: tentative -> confirmed -> coasting -> dropped.
//
//   Tentative  started by one position-observing measurement. Confirmed after
//              `confirm_hits` position-observing updates (including the first),
//              none more than `tentative_timeout_s` apart; dropped as soon as
//              it goes `tentative_timeout_s` without an update.
//   Confirmed  reported to operators. Becomes Coasting after `coast_after_s`
//              without any update.
//   Coasting   still reported (flagged) and still associated; a new update
//              returns it to Confirmed. Dropped after `drop_after_s` without
//              an update.
//   Dropped    removed from the live table (kept briefly in the store's history
//              so its trace can still be queried).
//
// The timers count from the last *position-observing* update (range-bearing
// or geodetic position). Bearing-only updates refine a track but neither
// confirm it, keep it out of coasting, nor keep it alive: one bearing cannot
// fix range, so a track held only by bearings from one site would drift in
// range while still looking confirmed.
//
// Any live track is also dropped if its predicted 1-sigma position
// uncertainty exceeds `max_position_sigma_m`: a track that vague cannot be
// associated reliably and would capture unrelated measurements.
//
// All times are measurement times (the engine's clock is the latest accepted
// measurement time), never the wall clock, so lifecycle decisions replay
// exactly. Defaults and their reasoning: docs/design.md section 6.
namespace sifted::association {

enum class TrackStatus : std::uint8_t { Tentative, Confirmed, Coasting, Dropped };

std::string_view to_string(TrackStatus s) noexcept;

enum class DropReason : std::uint8_t {
  None,
  TentativeTimeout,  // tentative and not updated for tentative_timeout_s
  Stale,             // confirmed/coasting and not updated for drop_after_s
  Uncertainty,       // predicted position sigma above max_position_sigma_m
};

std::string_view to_string(DropReason r) noexcept;

struct LifecycleConfig {
  // Position-observing hits needed to confirm. With detection probability
  // ~0.9 three hits arrive in ~3 revisits; a false alarm would have to be
  // followed by two more gated false alarms, which is rare at modest clutter.
  std::uint32_t confirm_hits = 3;
  // Default revisit assumption: sensors around 1 Hz. 3 s tolerates two
  // consecutive misses before a tentative track is discarded.
  double tentative_timeout_s = 3.0;
  // One missed 1 Hz revisit is normal (2 s gap); coasting starts beyond 2.5 s.
  double coast_after_s = 2.5;
  // Long enough to bridge short outages and crossings, short enough that a
  // departed target's track does not linger (~10 revisits).
  double drop_after_s = 10.0;
  // Well above the 20-50 m errors of the simulated sensors, well below typical
  // target separations of kilometres.
  double max_position_sigma_m = 1000.0;

  friend bool operator==(const LifecycleConfig&, const LifecycleConfig&) = default;
};

// True iff every field is finite and positive, confirm_hits >= 1,
// coast_after_s <= drop_after_s, and drop/tentative times are below the
// filter's longest prediction step.
bool is_valid(const LifecycleConfig& c) noexcept;

}  // namespace sifted::association
