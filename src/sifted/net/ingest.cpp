#include "sifted/net/ingest.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "sifted/net/snapshot.hpp"

namespace sifted::net {
namespace {

std::string token(std::string_view s) {
  std::string out(s);
  std::replace(out.begin(), out.end(), ' ', '_');
  return out;
}

double seconds_between(Timestamp from, Timestamp to) noexcept {
  const auto diff = static_cast<std::uint64_t>(to.ns) - static_cast<std::uint64_t>(from.ns);
  return static_cast<double>(diff) * 1e-9;
}

}  // namespace

bool is_valid(const PipelineConfig& c) noexcept {
  return sifted::is_valid(c.engine) && c.snapshot_interval_s >= 0.0 &&
         std::isfinite(c.snapshot_interval_s) && c.max_scan_size >= 1 && c.max_scan_size <= 4096 &&
         c.dedup_window <= 65536 && c.reorder_window_s >= 0.0 && c.reorder_window_s <= 60.0 &&
         c.reorder_capacity >= 1 && c.reorder_capacity <= 65536 && c.max_time_ahead_s >= 0.0 &&
         c.max_time_ahead_s <= 86400.0;
}

Pipeline::Pipeline(SensorTable sensors, PipelineConfig config, LineSink output,
                   SnapshotSink publish)
    : config_(config),
      engine_(std::move(sensors), config.engine),
      dedup_(engine_.sensors(), config.dedup_window),
      output_(std::move(output)),
      publish_(std::move(publish)) {
  if (!is_valid(config_)) {
    throw std::invalid_argument("sifted::net::Pipeline: invalid PipelineConfig");
  }
  pending_.reserve(config_.max_scan_size);
  reorder_window_ns_ = std::llround(config_.reorder_window_s * 1e9);
  max_ahead_ns_ = std::llround(config_.max_time_ahead_s * 1e9);
}

void Pipeline::emit(std::string line) {
  if (output_) {
    line += '\n';
    output_(line);
  }
}

void Pipeline::on_frame(Timestamp recv, std::span<const std::uint8_t> frame) {
  ++frames_;
  const auto decoded = wire::decode_measurement(frame);
  if (!decoded.ok()) {
    ++decode_errors_[static_cast<std::size_t>(decoded.error())];
    emit("x " + std::to_string(recv.ns) + " " + token(wire::to_string(decoded.error())));
    return;
  }
  const Measurement& m = decoded.value();
  if (dedup_.is_duplicate(m.sensor_id, m.seq, m.time)) {
    emit("d " + std::to_string(recv.ns) + " " + std::to_string(m.sensor_id) + " " +
         std::to_string(m.seq) + " " + std::to_string(m.time.ns));
    return;
  }
  // Compared as a difference in uint64 so extreme timestamps cannot overflow.
  if (max_ahead_ns_ > 0 && m.time > recv &&
      static_cast<std::uint64_t>(m.time.ns) - static_cast<std::uint64_t>(recv.ns) >
          static_cast<std::uint64_t>(max_ahead_ns_)) {
    ++future_rejected_;
    emit("f " + std::to_string(recv.ns) + " " + std::to_string(m.sensor_id) + " " +
         std::to_string(m.seq) + " " + std::to_string(m.time.ns));
    return;
  }
  if (reorder_window_ns_ == 0) {
    assemble(recv, m);
    return;
  }
  if (released_ && m.time < *released_) {
    ++reorder_late_;  // its slot has already gone by: holding it cannot help
    assemble(recv, m);
    release(recv, false);
    return;
  }
  const Held h{m, recv, arrivals_++};
  const auto pos =
      std::upper_bound(held_.begin(), held_.end(), h, [](const Held& a, const Held& b) {
        if (a.m.time != b.m.time) {
          return a.m.time < b.m.time;
        }
        if (a.m.sensor_id != b.m.sensor_id) {
          return a.m.sensor_id < b.m.sensor_id;
        }
        return a.order < b.order;
      });
  held_.insert(pos, h);
  held_high_water_ = std::max(held_high_water_, held_.size());
  release(recv, false);
}

void Pipeline::assemble(Timestamp recv, const Measurement& m) {
  if (!pending_.empty() &&
      (pending_.front().sensor_id != m.sensor_id || pending_.front().time != m.time ||
       pending_.size() >= config_.max_scan_size)) {
    flush(recv);
  }
  pending_.push_back(m);
}

void Pipeline::release(Timestamp now, bool all) {
  if (held_.empty()) {
    return;
  }
  std::optional<Timestamp> watermark;
  if (all) {
    watermark = held_.back().m.time;
  } else {
    for (const Held& h : held_) {
      // Expired: received at least the window ago (h.recv <= now - window).
      const bool expired = now >= h.recv && static_cast<std::uint64_t>(now.ns) -
                                                    static_cast<std::uint64_t>(h.recv.ns) >=
                                                static_cast<std::uint64_t>(reorder_window_ns_);
      if (expired && (!watermark || h.m.time > *watermark)) {
        watermark = h.m.time;
      }
    }
  }
  std::size_t n = 0;
  if (watermark) {
    while (n < held_.size() && held_[n].m.time <= *watermark) {
      ++n;
    }
  }
  if (held_.size() - n > config_.reorder_capacity) {
    const std::size_t forced = held_.size() - n - config_.reorder_capacity;
    reorder_forced_ += forced;
    n += forced;
  }
  for (std::size_t i = 0; i < n; ++i) {
    released_ = held_[i].m.time;
    assemble(now, held_[i].m);
  }
  held_.erase(held_.begin(), held_.begin() + static_cast<std::ptrdiff_t>(n));
}

void Pipeline::on_tick(Timestamp recv) {
  release(recv, false);
  flush(recv);
}

void Pipeline::finish(Timestamp recv) {
  release(recv, true);
  flush(recv);
}

void Pipeline::flush(Timestamp recv) {
  if (pending_.empty()) {
    return;
  }
  ++scans_;
  results_.clear();
  engine_.on_scan(pending_, results_);
  for (std::size_t i = 0; i < pending_.size(); ++i) {
    const Measurement& m = pending_[i];
    emit("m " + std::to_string(recv.ns) + " " + std::to_string(m.sensor_id) + " " +
         std::to_string(m.seq) + " " + std::to_string(m.time.ns) + " " +
         token(to_string(results_[i].outcome)) + " " + std::to_string(results_[i].track));
  }
  pending_.clear();

  const auto now = engine_.time();
  if (now &&
      (!last_snapshot_ || seconds_between(*last_snapshot_, *now) >= config_.snapshot_interval_s)) {
    last_snapshot_ = now;
    const std::string json =
        format_snapshot(++snapshot_seq_, *now, engine_.tracks_at(*now), config_.engine.origin);
    emit("s " + json);
    if (publish_) {
      publish_(json);
    }
  }
}

}  // namespace sifted::net
