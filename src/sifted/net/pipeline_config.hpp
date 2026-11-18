#pragma once

#include <cstddef>
#include <cstdint>

#include "sifted/core/engine.hpp"

namespace sifted::net {

// Keeps a snapshot of the default 1024 tracks under the WebSocket server's
// 8 MiB message limit even with every excerpt full (about 7.4 MiB at worst).
inline constexpr std::size_t kMaxSnapshotTraceEvents = 32;

// Everything besides the sensor table that determines the output of the
// ingest -> engine path. Recorded in full in every event log header so a
// replay is configured exactly like the run it reproduces.
struct PipelineConfig {
  EngineConfig engine;
  // Track snapshots are published whenever the engine clock (measurement
  // time, never the wall clock) has advanced this far since the last one.
  double snapshot_interval_s = 0.5;
  // Newest decision-trace events published per track in each snapshot (0:
  // none). Display only: it changes the snapshot text, never the tracking.
  // Clients join successive excerpts, so it only has to cover the decisions
  // made between two snapshots (docs/design.md section 8).
  std::size_t snapshot_trace_events = 4;
  // Per-sensor duplicate-suppression window: the last N (seq, time) keys
  // accepted from each configured sensor.
  std::size_t dedup_window = 256;
  // Consecutive frames with the same sensor and time are associated as one
  // scan; a scan is cut at this size (bounds the assignment problem).
  std::size_t max_scan_size = 256;
  // Bounded reordering stage (ADR 0009). Decoded frames are held for up to
  // reorder_window_s of *receive* time and released in measurement-time
  // order, so delay jitter does not turn into out-of-order drops or split
  // scans. 0 disables the stage: frames go to scan assembly in arrival order
  // (the M2 behaviour). At most reorder_capacity frames are held; beyond that
  // the oldest measurement is released early (counted).
  double reorder_window_s = 0.0;
  std::size_t reorder_capacity = 4096;
  // Clock sanity gate (ADR 0010). A frame whose measurement time is more than
  // max_time_ahead_s ahead of its receive time is rejected before it can move
  // the engine clock (one sensor with a fast clock would otherwise make every
  // other sensor's data OutOfOrder). Needs sensors and receiver on a common
  // timebase (wall-clock senders, or simulation), so 0 disables it.
  double max_time_ahead_s = 0.0;
  // Timeline guard (ADR 0012). Needs no common timebase: a frame is judged
  // against the measurement times already accepted. It is on the current
  // timeline if it is at most max_time_jump_s behind the newest accepted time,
  // or ahead of it by at most max_time_jump_s plus the receive time elapsed
  // since. Anything else is quarantined and reaches nothing downstream. A
  // quarantined timeline replaces the current one only after
  // timeline_confirm_frames distinct, mutually consistent frames AND
  // timeline_confirm_s of receive time in which the current timeline received
  // no frame. 0 disables the guard (the behaviour of older recordings); the
  // daemon defaults both durations to coast_after_s: a jump that small cannot
  // even make a track coast.
  double max_time_jump_s = 0.0;
  std::uint32_t timeline_confirm_frames = 3;
  double timeline_confirm_s = 2.5;
  // Feed staleness (ADR 0012). A sensor is stale when nothing has been
  // accepted from it for max(feed_stale_min_s, feed_stale_factor x the median
  // of its last five inter-scan receive gaps), so the threshold follows each
  // sensor's own cadence. 0 disables the feature and leaves snapshots exactly
  // as older recordings have them. Display only: it never changes tracking.
  double feed_stale_factor = 0.0;
  double feed_stale_min_s = 2.5;

  friend bool operator==(const PipelineConfig&, const PipelineConfig&) = default;
};

bool is_valid(const PipelineConfig& c) noexcept;

}  // namespace sifted::net
