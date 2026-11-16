#pragma once

#include <cstddef>

#include "sifted/core/engine.hpp"

namespace sifted::net {

// Everything besides the sensor table that determines the output of the
// ingest -> engine path. Recorded in full in every event log header so a
// replay is configured exactly like the run it reproduces.
struct PipelineConfig {
  EngineConfig engine;
  // Track snapshots are published whenever the engine clock (measurement
  // time, never the wall clock) has advanced this far since the last one.
  double snapshot_interval_s = 0.5;
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

  friend bool operator==(const PipelineConfig&, const PipelineConfig&) = default;
};

bool is_valid(const PipelineConfig& c) noexcept;

}  // namespace sifted::net
