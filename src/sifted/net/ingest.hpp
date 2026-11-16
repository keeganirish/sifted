#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/core/types.hpp"
#include "sifted/net/dedup.hpp"
#include "sifted/net/pipeline_config.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::net {

// The deterministic half of ingest, shared by the live daemon and the
// replayer: everything after a frame has been received and time-stamped.
//
//   bytes -> strict wire decode -> duplicate suppression -> clock gate
//         -> [reorder stage] -> scan assembly -> Engine::on_scan
//         -> output lines (+ periodic track snapshots)
//
// It runs on the engine thread only. Its output is a pure function of the
// sensor table, the PipelineConfig and the ordered (recv time, frame | tick)
// sequence it is given, which is exactly what the event log records; replaying
// a log through a fresh Pipeline therefore reproduces the output byte for byte.
//
// Scan assembly: consecutive frames with the same sensor and measurement time
// are held back and associated together (one radar scan). A pending scan is
// flushed by the next frame with a different key, by reaching max_scan_size,
// by a tick (the live input went quiet) or by finish().
//
// Output lines (each ends in '\n'):
//   m <recv_ns> <sensor> <seq> <time_ns> <outcome> <track>   one per measurement
//   x <recv_ns> <decode-error>                               frame failed to decode
//   d <recv_ns> <sensor> <seq> <time_ns>                     duplicate suppressed
//   f <recv_ns> <sensor> <seq> <time_ns>                     too far ahead of recv (clock gate)
//   s <snapshot json>                                        track snapshot
// <outcome> and <decode-error> are to_string() with spaces replaced by '_'.
class Pipeline {
 public:
  using LineSink = std::function<void(std::string_view line)>;
  using SnapshotSink = std::function<void(const std::string& json)>;

  // Throws std::invalid_argument if !is_valid(config).
  Pipeline(SensorTable sensors, PipelineConfig config, LineSink output = {},
           SnapshotSink publish = {});

  void on_frame(Timestamp recv, std::span<const std::uint8_t> frame);
  void on_tick(Timestamp recv);
  void finish(Timestamp recv);

  // A scan is pending or frames are held in the reorder stage: a later tick
  // or finish() will produce output.
  bool has_pending() const noexcept { return !pending_.empty() || !held_.empty(); }
  const Engine& engine() const noexcept { return engine_; }
  const PipelineConfig& config() const noexcept { return config_; }
  const SensorTable& sensors() const noexcept { return engine_.sensors(); }

  std::uint64_t frames() const noexcept { return frames_; }
  std::uint64_t decode_errors(wire::DecodeError e) const noexcept {
    return decode_errors_[static_cast<std::size_t>(e)];
  }
  std::uint64_t duplicates() const noexcept { return dedup_.suppressed(); }
  std::uint64_t scans() const noexcept { return scans_; }
  std::uint64_t snapshots() const noexcept { return snapshot_seq_; }
  // Clock gate rejections (max_time_ahead_s).
  std::uint64_t future_rejected() const noexcept { return future_rejected_; }
  // Reorder stage: frames held now, the most ever held, frames released early
  // because the stage was full, and late frames that bypassed it.
  std::size_t held() const noexcept { return held_.size(); }
  std::size_t held_high_water() const noexcept { return held_high_water_; }
  std::uint64_t reorder_forced() const noexcept { return reorder_forced_; }
  std::uint64_t reorder_late() const noexcept { return reorder_late_; }

 private:
  struct Held {
    Measurement m;
    Timestamp recv;
    std::uint64_t order;  // arrival counter: stable tie-break
  };
  void assemble(Timestamp recv, const Measurement& m);
  void release(Timestamp now, bool all);
  void flush(Timestamp recv);
  void emit(std::string line);

  PipelineConfig config_;
  Engine engine_;
  DuplicateFilter dedup_;
  LineSink output_;
  SnapshotSink publish_;
  std::vector<Measurement> pending_;
  std::vector<MeasurementResult> results_;
  std::optional<Timestamp> last_snapshot_;
  std::uint64_t snapshot_seq_ = 0;
  std::uint64_t frames_ = 0;
  std::uint64_t scans_ = 0;
  std::array<std::uint64_t, 9> decode_errors_{};
  std::int64_t reorder_window_ns_ = 0;
  std::int64_t max_ahead_ns_ = 0;
  std::vector<Held> held_;  // sorted by (time, sensor, order)
  std::uint64_t arrivals_ = 0;
  std::optional<Timestamp> released_;  // latest measurement time released
  std::size_t held_high_water_ = 0;
  std::uint64_t reorder_forced_ = 0;
  std::uint64_t reorder_late_ = 0;
  std::uint64_t future_rejected_ = 0;
};

}  // namespace sifted::net
