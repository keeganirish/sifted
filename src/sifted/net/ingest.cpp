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
         c.dedup_window <= 65536;
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
  if (!pending_.empty() &&
      (pending_.front().sensor_id != m.sensor_id || pending_.front().time != m.time ||
       pending_.size() >= config_.max_scan_size)) {
    flush(recv);
  }
  pending_.push_back(m);
}

void Pipeline::on_tick(Timestamp recv) { flush(recv); }

void Pipeline::finish(Timestamp recv) { flush(recv); }

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
