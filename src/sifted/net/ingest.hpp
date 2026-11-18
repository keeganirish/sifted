#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/core/types.hpp"
#include "sifted/net/dedup.hpp"
#include "sifted/net/pipeline_config.hpp"
#include "sifted/net/snapshot.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::net {

// The deterministic half of ingest, shared by the live daemon and the
// replayer: everything after a frame has been received and time-stamped.
//
//   bytes -> strict wire decode -> timeline guard -> duplicate suppression
//         -> clock gate -> [reorder stage] -> scan assembly -> Engine::on_scan
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
// Timeline guard (ADR 0012, off unless max_time_jump_s > 0): frames off the
// current timeline are quarantined, so they cannot move the engine clock, touch
// a pending scan, enter the reorder stage or occupy the duplicate filter. See
// PipelineConfig for when a quarantined timeline is adopted.
//
// Silence (ADR 0012, off unless feed_stale_factor > 0): a tick also refreshes
// the feed status. During total silence it publishes predicted snapshots built
// by Engine::tracks_if_silent_until, which leave the tracker untouched: ticks
// can change what is displayed, never what is tracked.
//
// Output lines (each ends in '\n'):
//   m <recv_ns> <sensor> <seq> <time_ns> <outcome> <track>   one per measurement
//   x <recv_ns> <decode-error>                               frame failed to decode
//   d <recv_ns> <sensor> <seq> <time_ns>                     duplicate suppressed
//   f <recv_ns> <sensor> <seq> <time_ns>                     too far ahead of recv (clock gate)
//   t <recv_ns> <sensor> <seq> <time_ns>                     rejected by the timeline guard
//   T <recv_ns> <newest_time_ns> <frames> <restarted 0|1>    a new timeline was adopted
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
  // When a tick would next change something although nothing is pending: a
  // quarantined timeline can be adopted, a sensor goes stale, or (every
  // `period_ns` while predicted tracks are on display) the picture moves. None
  // once the picture can no longer change, so a silent run stops ticking and
  // its log stops growing.
  std::optional<Timestamp> next_idle_tick(std::int64_t period_ns) const;

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
  // Timeline guard: frames finally refused, frames in quarantine now, timelines
  // adopted after start-up, and how many of those restarted the tracker.
  std::uint64_t timeline_rejected() const noexcept { return timeline_rejected_; }
  std::size_t timeline_held() const noexcept;
  std::uint64_t timeline_switches() const noexcept { return timeline_switches_; }
  std::uint64_t timeline_restarts() const noexcept { return timeline_restarts_; }
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
  // A timeline that frames agree on but that has not been adopted.
  struct Candidate {
    Timestamp head;       // newest measurement time
    Timestamp head_recv;  // when it was received
    Timestamp first_recv;
    std::vector<Held> frames;  // distinct frames, in arrival order
    std::set<std::tuple<SensorId, std::uint32_t, std::int64_t>> keys;
  };
  // Per-sensor receive history behind the staleness threshold.
  struct SensorActivity {
    std::optional<Timestamp> last_recv;
    std::optional<Timestamp> last_time;
    Timestamp scan_recv;
    std::array<std::int64_t, 5> gaps{};  // ring of inter-scan receive gaps
    std::size_t gap_count = 0;
    std::size_t gap_next = 0;
    bool stale = false;  // as of the last feed status
  };

  bool too_far_ahead(Timestamp recv, const Measurement& m) const noexcept;
  bool on_timeline(Timestamp head, Timestamp head_recv, Timestamp t, Timestamp recv) const noexcept;
  bool passes_timeline_guard(Timestamp recv, const Measurement& m);
  void quarantine(Timestamp recv, const Measurement& m);
  std::optional<Timestamp> adoption_time(const Candidate& c) const noexcept;
  void settle_timeline(Timestamp recv);
  void adopt(std::size_t candidate, Timestamp recv);
  void reject(Timestamp recv, const Measurement& m);
  void discard_candidates(Timestamp recv);

  void admit(Timestamp recv, const Measurement& m);
  void assemble(Timestamp recv, const Measurement& m);
  void release(Timestamp now, bool all);
  void flush(Timestamp recv);
  void emit(std::string line);

  void note_activity(Timestamp recv, const Measurement& m);
  std::int64_t stale_after_ns(const SensorActivity& a) const noexcept;
  FeedStatus feed_status(Timestamp recv);
  void publish_silence(Timestamp recv);
  void publish_snapshot(Timestamp time, const std::vector<TrackEstimate>& tracks,
                        const std::optional<FeedStatus>& feed);

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

  std::int64_t max_jump_ns_ = 0;  // 0: the timeline guard is off
  std::int64_t confirm_ns_ = 0;
  std::optional<Timestamp> timeline_head_;  // newest time on the current timeline
  Timestamp timeline_head_recv_;
  Timestamp timeline_support_recv_;  // last frame received on the current timeline
  std::vector<Candidate> candidates_;
  std::uint64_t timeline_rejected_ = 0;
  std::uint64_t timeline_switches_ = 0;
  std::uint64_t timeline_restarts_ = 0;

  std::int64_t stale_min_ns_ = 0;
  std::vector<SensorActivity> activity_;  // indexed like the sensor table; empty when off
  std::optional<FeedState> feed_shown_;
  std::optional<Timestamp> silence_published_;
  bool predicting_ = false;  // predicted tracks are on display
};

}  // namespace sifted::net
