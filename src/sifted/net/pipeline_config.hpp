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

  friend bool operator==(const PipelineConfig&, const PipelineConfig&) = default;
};

bool is_valid(const PipelineConfig& c) noexcept;

}  // namespace sifted::net
