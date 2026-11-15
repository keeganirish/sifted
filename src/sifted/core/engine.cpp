#include "sifted/core/engine.hpp"

#include <utility>

#include "sifted/estimation/range_bearing.hpp"

namespace sifted {
namespace {

using estimation::FilterStatus;
using estimation::RangeBearing;

constexpr double kNsToS = 1e-9;

// Elapsed seconds from `from` to `to`; precondition from <= to. The subtraction
// is done in uint64 so extreme (but valid) timestamps cannot overflow int64.
double seconds_between(Timestamp from, Timestamp to) noexcept {
  const auto diff = static_cast<std::uint64_t>(to.ns) - static_cast<std::uint64_t>(from.ns);
  return static_cast<double>(diff) * kNsToS;
}

RangeBearing z_of(const Measurement& m) noexcept { return {m.z[0], m.z[1]}; }
RangeBearing sigma_of(const Measurement& m) noexcept { return {m.sigma[0], m.sigma[1]}; }

}  // namespace

std::string_view to_string(MeasurementOutcome o) noexcept {
  switch (o) {
    case MeasurementOutcome::Initialized:
      return "initialized";
    case MeasurementOutcome::Updated:
      return "updated";
    case MeasurementOutcome::Invalid:
      return "invalid";
    case MeasurementOutcome::UnknownSensor:
      return "unknown sensor";
    case MeasurementOutcome::KindMismatch:
      return "kind mismatch";
    case MeasurementOutcome::UnsupportedKind:
      return "unsupported kind";
    case MeasurementOutcome::Duplicate:
      return "duplicate";
    case MeasurementOutcome::OutOfOrder:
      return "out of order";
    case MeasurementOutcome::FilterRejected:
      return "filter rejected";
  }
  return "unknown outcome";
}

Engine::Engine(SensorTable sensors, EngineConfig config)
    : sensors_(std::move(sensors)), config_(config), last_seq_(sensors_.size()) {}

MeasurementOutcome Engine::on_measurement(const Measurement& m) noexcept {
  ++processed_;
  const MeasurementOutcome o = process(m);
  ++outcome_counts_[static_cast<std::size_t>(o)];
  return o;
}

MeasurementOutcome Engine::process(const Measurement& m) noexcept {
  if (!is_valid(m)) {
    return MeasurementOutcome::Invalid;
  }
  const auto index = sensors_.index_of(m.sensor_id);
  if (!index) {
    return MeasurementOutcome::UnknownSensor;
  }
  const SensorConfig& sensor = sensors_.sensors()[*index];
  if (m.kind != sensor.kind) {
    return MeasurementOutcome::KindMismatch;
  }
  if (m.kind != SensorKind::RangeBearing) {
    return MeasurementOutcome::UnsupportedKind;
  }
  if (last_seq_[*index] == m.seq) {
    return MeasurementOutcome::Duplicate;
  }

  MeasurementOutcome outcome = MeasurementOutcome::FilterRejected;
  if (!track_) {
    outcome = initialize(m, sensor);
  } else if (m.time < track_->time) {
    return MeasurementOutcome::OutOfOrder;
  } else {
    // Work on a copy and commit only if predict *and* update succeed, so a
    // rejected measurement leaves the track exactly as it was.
    estimation::ConstantVelocityFilter f = track_->filter;
    const double dt = seconds_between(track_->time, m.time);
    if (f.predict(dt) == FilterStatus::Ok &&
        estimation::update_range_bearing(f, sensor.pose, z_of(m), sigma_of(m)) ==
            FilterStatus::Ok) {
      track_->filter = f;
      track_->time = m.time;
      ++track_->updates;
      track_->last_sensor = m.sensor_id;
      outcome = MeasurementOutcome::Updated;
    }
  }

  if (outcome == MeasurementOutcome::Initialized || outcome == MeasurementOutcome::Updated) {
    last_seq_[*index] = m.seq;
  }
  return outcome;
}

MeasurementOutcome Engine::initialize(const Measurement& m, const SensorConfig& sensor) noexcept {
  const estimation::LocalPosition p =
      estimation::range_bearing_to_local(z_of(m), sigma_of(m), sensor.pose);

  estimation::StateVector x0 = estimation::StateVector::Zero();
  x0(0) = p.position(0);
  x0(1) = p.position(1);

  estimation::StateCovariance P0 = estimation::StateCovariance::Zero();
  P0.topLeftCorner<2, 2>() = p.covariance;
  const double v_var = config_.initial_velocity_sigma * config_.initial_velocity_sigma;
  P0(2, 2) = v_var;
  P0(3, 3) = v_var;

  auto created = estimation::ConstantVelocityFilter::create(x0, P0, config_.accel_psd);
  if (!created) {
    return MeasurementOutcome::FilterRejected;  // e.g. zero range: degenerate covariance
  }
  track_.emplace(Track{next_track_id_++, created.value(), m.time, 1, m.sensor_id});
  return MeasurementOutcome::Initialized;
}

std::optional<TrackEstimate> Engine::track() const {
  if (!track_) {
    return std::nullopt;
  }
  TrackEstimate e;
  e.id = track_->id;
  e.time = track_->time;
  e.state = track_->filter.state();
  e.covariance = track_->filter.covariance();
  e.updates = track_->updates;
  e.last_sensor = track_->last_sensor;
  return e;
}

std::optional<TrackEstimate> Engine::track_at(Timestamp t) const {
  if (!track_ || t < track_->time) {
    return std::nullopt;
  }
  estimation::ConstantVelocityFilter f = track_->filter;
  if (f.predict(seconds_between(track_->time, t)) != FilterStatus::Ok) {
    return std::nullopt;
  }
  std::optional<TrackEstimate> e = track();
  e->time = t;
  e->state = f.state();
  e->covariance = f.covariance();
  return e;
}

}  // namespace sifted
