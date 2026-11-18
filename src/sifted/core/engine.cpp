#include "sifted/core/engine.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "sifted/association/assignment.hpp"
#include "sifted/association/gating.hpp"
#include "sifted/estimation/measurement_model.hpp"

namespace sifted {
namespace {

using association::TrackStatus;
using estimation::FilterStatus;
using store::TraceEvent;
using store::TraceEventKind;
using store::TrackRecord;

constexpr double kNsToS = 1e-9;

// Elapsed seconds from `from` to `to`; precondition from <= to. The subtraction
// is done in uint64 so extreme (but valid) timestamps cannot overflow int64.
double seconds_between(Timestamp from, Timestamp to) noexcept {
  const auto diff = static_cast<std::uint64_t>(to.ns) - static_cast<std::uint64_t>(from.ns);
  return static_cast<double>(diff) * kNsToS;
}

// Largest eigenvalue of the 2x2 position block of P (closed form).
double max_position_variance(const estimation::StateCovariance& P) noexcept {
  const double a = P(0, 0);
  const double b = P(0, 1);
  const double d = P(1, 1);
  const double half_trace = 0.5 * (a + d);
  const double disc = std::sqrt(0.25 * (a - d) * (a - d) + b * b);
  return half_trace + disc;
}

bool positive_finite(double v) noexcept { return v > 0.0 && std::isfinite(v); }

// One candidate (measurement, track) pairing inside a group.
struct Candidate {
  std::size_t meas;   // index into the group's accepted measurements
  std::size_t track;  // index into store_.live() at association time
  double d2;
  bool gated;
  estimation::AnyLinearized lin;
};

// Degrees of freedom of the chi-square approximation to the fading-memory NIS
// sum for 2-D measurements: n (1 + alpha) / (1 - alpha), rounded.
std::size_t manoeuvre_dof(double alpha) noexcept {
  return static_cast<std::size_t>(std::lround(2.0 * (1.0 + alpha) / (1.0 - alpha)));
}

bool is_valid(const ManoeuvreConfig& m) noexcept {
  return m.fading > 0.0 && m.fading < 1.0 &&
         manoeuvre_dof(m.fading) <= association::kMaxChiSquareDof && m.detect_probability > 0.0 &&
         m.detect_probability < 1.0 && m.accel_psd_factor >= 1.0 &&
         std::isfinite(m.accel_psd_factor);
}

}  // namespace

bool is_valid(const EngineConfig& c) noexcept {
  const double p = c.association.gate_probability;
  return c.accel_psd >= 0.0 && std::isfinite(c.accel_psd) && is_valid(c.manoeuvre) &&
         std::isfinite(c.accel_psd * c.manoeuvre.accel_psd_factor) &&
         positive_finite(c.initial_velocity_sigma) && p > 0.0 && p < 1.0 &&
         c.association.near_miss_factor >= 1.0 && std::isfinite(c.association.near_miss_factor) &&
         (c.association.method == association::AssignmentMethod::GlobalNearestNeighbour ||
          c.association.method == association::AssignmentMethod::Greedy) &&
         association::is_valid(c.lifecycle) && c.max_tracks >= 1 &&
         (!c.origin || estimation::is_valid(*c.origin));
}

Engine::Engine(SensorTable sensors, EngineConfig config)
    : sensors_(std::move(sensors)),
      config_(config),
      store_(config.trace),
      last_seq_(sensors_.size()) {
  if (!is_valid(config_)) {
    throw std::invalid_argument("sifted::Engine: invalid EngineConfig");
  }
  for (std::size_t dim = 1; dim <= 2; ++dim) {
    const auto g = association::chi_square_quantile(dim, config_.association.gate_probability);
    if (!g) {
      throw std::invalid_argument("sifted::Engine: gate probability not representable");
    }
    gates_[dim - 1] = *g;
  }
  const ManoeuvreConfig& mc = config_.manoeuvre;
  const auto q = association::chi_square_quantile(manoeuvre_dof(mc.fading), mc.detect_probability);
  if (!q) {
    throw std::invalid_argument("sifted::Engine: manoeuvre threshold not representable");
  }
  manoeuvre_on_ = *q / (1.0 + mc.fading);
  manoeuvre_off_ = 2.0 / (1.0 - mc.fading);
}

MeasurementOutcome Engine::on_measurement(const Measurement& m) {
  return on_measurement_result(m).outcome;
}

MeasurementResult Engine::on_measurement_result(const Measurement& m) {
  std::vector<MeasurementResult> out;
  process_group(std::span<const Measurement>(&m, 1), out);
  return out.front();
}

void Engine::on_scan(std::span<const Measurement> scan, std::vector<MeasurementResult>& out) {
  std::size_t begin = 0;
  while (begin < scan.size()) {
    std::size_t end = begin + 1;
    while (end < scan.size() && scan[end].sensor_id == scan[begin].sensor_id &&
           scan[end].time == scan[begin].time) {
      ++end;
    }
    process_group(scan.subspan(begin, end - begin), out);
    begin = end;
  }
}

void Engine::maintain(Timestamp t) {
  const association::LifecycleConfig& lc = config_.lifecycle;
  const double max_var = lc.max_position_sigma_m * lc.max_position_sigma_m;
  std::vector<std::pair<TrackId, association::DropReason>> drops;
  for (TrackRecord& r : store_.live()) {
    const double age = seconds_between(r.last_position_time, t);
    association::DropReason reason = association::DropReason::None;
    if (r.status == TrackStatus::Tentative) {
      if (age > lc.tentative_timeout_s) {
        reason = association::DropReason::TentativeTimeout;
      }
    } else if (age > lc.drop_after_s) {
      reason = association::DropReason::Stale;
    }
    if (reason == association::DropReason::None) {
      estimation::ConstantVelocityFilter f = r.filter;
      if (f.predict(seconds_between(r.time, t)) != FilterStatus::Ok ||
          !(max_position_variance(f.covariance()) <= max_var)) {
        reason = association::DropReason::Uncertainty;
      }
    }
    if (reason != association::DropReason::None) {
      drops.emplace_back(r.id, reason);
      continue;
    }
    if (r.status == TrackStatus::Confirmed && age > lc.coast_after_s) {
      r.status = TrackStatus::Coasting;
      TraceEvent e;
      e.kind = TraceEventKind::Coasting;
      e.time = t;
      store_.record(r, e);
    }
  }
  for (const auto& [id, reason] : drops) {  // in id order
    store_.drop(id, t, reason);
    ++drop_counts_[static_cast<std::size_t>(reason)];
  }
}

void Engine::process_group(std::span<const Measurement> group,
                           std::vector<MeasurementResult>& out) {
  const std::size_t base = out.size();
  out.resize(base + group.size());

  // ---- 1. screen --------------------------------------------------------
  struct Accepted {
    std::size_t index;         // position in `group`
    std::size_t sensor_index;  // position in sensors_
    estimation::MeasurementContext ctx;
  };
  std::vector<Accepted> accepted;
  accepted.reserve(group.size());
  for (std::size_t i = 0; i < group.size(); ++i) {
    const Measurement& m = group[i];
    MeasurementResult& res = out[base + i];
    if (!is_valid(m)) {
      res.outcome = MeasurementOutcome::Invalid;
      continue;
    }
    const auto index = sensors_.index_of(m.sensor_id);
    if (!index) {
      res.outcome = MeasurementOutcome::UnknownSensor;
      continue;
    }
    const SensorConfig& sensor = sensors_.sensors()[*index];
    if (m.kind != sensor.kind) {
      res.outcome = MeasurementOutcome::KindMismatch;
      continue;
    }
    if (m.kind == SensorKind::GeoPosition && !config_.origin) {
      res.outcome = MeasurementOutcome::UnsupportedKind;
      continue;
    }
    const auto& last = last_seq_[*index];
    const bool dup_of_last = last && last->first == m.seq && last->second == m.time.ns;
    const bool dup_in_group = std::any_of(accepted.begin(), accepted.end(), [&](const Accepted& a) {
      return group[a.index].sensor_id == m.sensor_id && group[a.index].seq == m.seq &&
             group[a.index].time == m.time;
    });
    if (dup_of_last || dup_in_group) {
      res.outcome = MeasurementOutcome::Duplicate;
      continue;
    }
    if (now_ && m.time < *now_) {
      res.outcome = MeasurementOutcome::OutOfOrder;
      continue;
    }
    auto ctx = estimation::make_context(m, sensor.pose, config_.origin);
    if (!ctx) {
      res.outcome = MeasurementOutcome::OutOfFrame;  // origin present, projection refused
      continue;
    }
    accepted.push_back({i, *index, *ctx});
  }

  if (!accepted.empty()) {
    // All measurements of a group share one timestamp.
    const Timestamp t = group[accepted.front().index].time;
    now_ = t;

    // ---- 2. lifecycle at the group time ---------------------------------
    maintain(t);

    // ---- 3. gating ------------------------------------------------------
    std::vector<TrackRecord>& live = store_.live();
    std::vector<std::optional<estimation::ConstantVelocityFilter>> predicted;
    predicted.reserve(live.size());
    for (const TrackRecord& r : live) {
      estimation::ConstantVelocityFilter f = r.filter;
      if (f.predict(seconds_between(r.time, t)) == FilterStatus::Ok) {
        predicted.emplace_back(f);
      } else {
        predicted.emplace_back(std::nullopt);
      }
    }

    const double near_factor = config_.association.near_miss_factor;
    // Per-track inputs of the gate screen, computed once per group.
    const bool screen = config_.association.gate_screen;
    std::vector<double> lambda(screen ? live.size() : 0, 0.0);
    std::vector<Eigen::Vector2d> predicted_pos(screen ? live.size() : 0, Eigen::Vector2d::Zero());
    for (std::size_t k = 0; screen && k < live.size(); ++k) {
      if (predicted[k]) {
        lambda[k] = max_position_variance(predicted[k]->covariance());
        predicted_pos[k] = predicted[k]->state().head<2>();
      }
    }
    std::vector<Candidate> candidates;
    for (std::size_t j = 0; j < accepted.size(); ++j) {
      const estimation::GateScreen gs =
          screen ? estimation::make_screen(accepted[j].ctx) : estimation::GateScreen{};
      // The largest limit any pair of this measurement is tested against.
      const double limit =
          near_factor * gate(accepted[j].ctx.m->kind == SensorKind::BearingOnly ? 1 : 2);
      for (std::size_t k = 0; k < live.size(); ++k) {
        if (!predicted[k]) {
          continue;
        }
        if (screen && !estimation::may_gate(gs, predicted_pos[k], lambda[k], limit)) {
          continue;
        }
        auto lin = estimation::linearize(accepted[j].ctx, predicted[k]->state());
        if (!lin) {
          continue;
        }
        const auto d2 = estimation::mahalanobis2(*predicted[k], *lin);
        if (!d2) {
          continue;
        }
        const double g = gate(estimation::dimension(*lin));
        if (*d2 <= near_factor * g) {
          candidates.push_back({j, k, *d2, *d2 <= g, *lin});
        }
      }
    }

    // ---- 4. two-pass global nearest neighbour ----------------------------
    constexpr std::size_t kNone = SIZE_MAX;
    std::vector<std::size_t> assigned(accepted.size(), kNone);  // -> index into live
    std::vector<std::size_t> cand_of(accepted.size(), kNone);   // -> index into candidates
    std::vector<char> track_taken(live.size(), 0);
    auto pass = [&](bool established) {
      std::vector<std::size_t> rows;  // accepted index per local row
      std::vector<std::size_t> row_of(accepted.size(), kNone);
      std::vector<std::size_t> cols;  // live index per local col
      std::vector<std::size_t> col_of(live.size(), kNone);
      std::vector<std::pair<std::size_t, std::size_t>> pairs;
      std::vector<std::size_t> pair_cand;
      for (std::size_t c = 0; c < candidates.size(); ++c) {
        const Candidate& cd = candidates[c];
        const bool is_established = live[cd.track].status != TrackStatus::Tentative;
        if (!cd.gated || is_established != established || assigned[cd.meas] != kNone ||
            track_taken[cd.track] != 0) {
          continue;
        }
        if (row_of[cd.meas] == kNone) {
          row_of[cd.meas] = rows.size();
          rows.push_back(cd.meas);
        }
        if (col_of[cd.track] == kNone) {
          col_of[cd.track] = cols.size();
          cols.push_back(cd.track);
        }
        pairs.emplace_back(row_of[cd.meas], col_of[cd.track]);
        pair_cand.push_back(c);
      }
      // Within each cluster, rows are ordered by position in the group and
      // columns by track id (live is sorted by id), so the solver's
      // lowest-index tie rule breaks ties by track id.
      for (const association::Component& comp :
           association::connected_components(pairs, rows.size(), cols.size())) {
        std::vector<std::size_t> comp_cols = comp.cols;
        std::sort(comp_cols.begin(), comp_cols.end(),
                  [&](std::size_t a, std::size_t b) { return cols[a] < cols[b]; });
        std::vector<std::size_t> comp_rows = comp.rows;
        std::sort(comp_rows.begin(), comp_rows.end(),
                  [&](std::size_t a, std::size_t b) { return rows[a] < rows[b]; });
        std::vector<double> cost(comp_rows.size() * comp_cols.size(), association::kForbidden);
        std::vector<std::size_t> cell_cand(cost.size(), kNone);
        double unassigned_cost = 0.0;
        for (std::size_t p = 0; p < pairs.size(); ++p) {
          const auto r_it = std::find(comp_rows.begin(), comp_rows.end(), pairs[p].first);
          const auto c_it = std::find(comp_cols.begin(), comp_cols.end(), pairs[p].second);
          if (r_it == comp_rows.end() || c_it == comp_cols.end()) {
            continue;
          }
          const auto cell = static_cast<std::size_t>(r_it - comp_rows.begin()) * comp_cols.size() +
                            static_cast<std::size_t>(c_it - comp_cols.begin());
          const Candidate& cd = candidates[pair_cand[p]];
          cost[cell] = cd.d2;
          cell_cand[cell] = pair_cand[p];
          unassigned_cost = std::max(unassigned_cost, gate(estimation::dimension(cd.lin)));
        }
        const std::vector<int> sol =
            config_.association.method == association::AssignmentMethod::Greedy
                ? association::solve_assignment_greedy(cost, comp_rows.size(), comp_cols.size(),
                                                       unassigned_cost)
                : association::solve_assignment(cost, comp_rows.size(), comp_cols.size(),
                                                unassigned_cost);
        for (std::size_t r = 0; r < comp_rows.size(); ++r) {
          if (sol[r] == association::kUnassigned) {
            continue;
          }
          const std::size_t cell = r * comp_cols.size() + static_cast<std::size_t>(sol[r]);
          const std::size_t meas = rows[comp_rows[r]];
          const std::size_t trk = cols[comp_cols[static_cast<std::size_t>(sol[r])]];
          assigned[meas] = trk;
          cand_of[meas] = cell_cand[cell];
          track_taken[trk] = 1;
        }
      }
    };
    pass(true);   // confirmed and coasting tracks first
    pass(false);  // then tentative tracks, with the measurements left over

    // ---- 5a. updates, in track-id order --------------------------------
    std::vector<std::size_t> by_track(live.size(), kNone);  // live index -> accepted index
    for (std::size_t j = 0; j < accepted.size(); ++j) {
      if (assigned[j] != kNone) {
        by_track[assigned[j]] = j;
      }
    }
    const association::LifecycleConfig& lc = config_.lifecycle;
    for (std::size_t k = 0; k < live.size(); ++k) {
      const std::size_t j = by_track[k];
      if (j == kNone) {
        continue;
      }
      TrackRecord& r = live[k];
      const Measurement& m = group[accepted[j].index];
      const Candidate& cd = candidates[cand_of[j]];
      MeasurementResult& res = out[base + accepted[j].index];
      TraceEvent e;
      e.time = t;
      e.sensor = m.sensor_id;
      e.seq = m.seq;
      e.distance2 = cd.d2;
      e.gate = gate(estimation::dimension(cd.lin));

      estimation::ConstantVelocityFilter f = *predicted[k];
      if (estimation::apply(f, cd.lin) != FilterStatus::Ok) {
        res.outcome = MeasurementOutcome::FilterRejected;
        e.kind = TraceEventKind::UpdateRejected;
        store_.record(r, e);
        continue;
      }
      r.filter = f;
      r.time = t;
      ++r.updates;
      r.last_sensor = m.sensor_id;
      e.kind = TraceEventKind::Associated;
      store_.record(r, e);
      const bool position = estimation::can_initiate(m.kind);
      if (position) {
        ++r.position_hits;
        r.last_position_time = t;
        update_manoeuvre(r, cd.d2, t);
      }
      store_.credit(r, m.sensor_id, t);
      res.outcome = MeasurementOutcome::Updated;
      res.track = r.id;

      if (r.status == TrackStatus::Tentative && r.position_hits >= lc.confirm_hits) {
        r.status = TrackStatus::Confirmed;
        r.confirmed_at = t;
        TraceEvent c;
        c.kind = TraceEventKind::Confirmed;
        c.time = t;
        store_.record(r, c);
      } else if (r.status == TrackStatus::Coasting && position) {
        r.status = TrackStatus::Confirmed;
        TraceEvent c;
        c.kind = TraceEventKind::Recovered;
        c.time = t;
        store_.record(r, c);
      }
    }

    // ---- 5b. trace the alternatives each track saw -----------------------
    for (const Candidate& cd : candidates) {
      if (assigned[cd.meas] == cd.track) {
        continue;
      }
      const Measurement& m = group[accepted[cd.meas].index];
      TraceEvent e;
      e.kind = cd.gated ? TraceEventKind::Competed : TraceEventKind::NearMiss;
      e.time = t;
      e.sensor = m.sensor_id;
      e.seq = m.seq;
      e.distance2 = cd.d2;
      e.gate = gate(estimation::dimension(cd.lin));
      if (assigned[cd.meas] != kNone) {
        e.other = live[assigned[cd.meas]].id;
      }
      store_.record(live[cd.track], e);
    }

    // ---- 5c. start tracks from what is left ------------------------------
    for (std::size_t j = 0; j < accepted.size(); ++j) {
      if (assigned[j] != kNone) {
        continue;
      }
      const Measurement& m = group[accepted[j].index];
      MeasurementResult& res = out[base + accepted[j].index];
      if (!estimation::can_initiate(m.kind)) {
        res.outcome = MeasurementOutcome::Unassociated;
        continue;
      }
      if (store_.size() >= config_.max_tracks) {
        res.outcome = MeasurementOutcome::TrackLimit;
        continue;
      }
      const auto p = estimation::initial_position(accepted[j].ctx);
      res.outcome = MeasurementOutcome::FilterRejected;
      if (!p) {
        continue;
      }
      estimation::StateVector x0 = estimation::StateVector::Zero();
      x0(0) = p->position(0);
      x0(1) = p->position(1);
      estimation::StateCovariance P0 = estimation::StateCovariance::Zero();
      P0.topLeftCorner<2, 2>() = p->covariance;
      const double v_var = config_.initial_velocity_sigma * config_.initial_velocity_sigma;
      P0(2, 2) = v_var;
      P0(3, 3) = v_var;
      auto created = estimation::ConstantVelocityFilter::create(x0, P0, config_.accel_psd);
      if (!created) {
        continue;  // e.g. zero range: degenerate covariance
      }
      TrackRecord r(next_track_id_++, created.value(), t, store_.config());
      r.updates = 1;
      r.position_hits = 1;
      r.last_sensor = m.sensor_id;
      TraceEvent e;
      e.kind = TraceEventKind::Initiated;
      e.time = t;
      e.sensor = m.sensor_id;
      e.seq = m.seq;
      store_.record(r, e);
      store_.credit(r, m.sensor_id, t);
      if (r.position_hits >= lc.confirm_hits) {
        r.status = TrackStatus::Confirmed;
        r.confirmed_at = t;
        TraceEvent c;
        c.kind = TraceEventKind::Confirmed;
        c.time = t;
        store_.record(r, c);
      }
      res.outcome = MeasurementOutcome::Initialized;
      res.track = r.id;
      store_.add(std::move(r));
    }

    for (const Accepted& a : accepted) {
      const MeasurementOutcome o = out[base + a.index].outcome;
      if (o == MeasurementOutcome::Initialized || o == MeasurementOutcome::Updated) {
        const Measurement& m = group[a.index];
        last_seq_[a.sensor_index] = std::make_pair(m.seq, m.time.ns);
      }
    }
  }

  for (std::size_t i = 0; i < group.size(); ++i) {
    const MeasurementResult& res = out[base + i];
    ++processed_;
    ++outcome_counts_[static_cast<std::size_t>(res.outcome)];
    store_.record_measurement(
        {group[i].time, group[i].sensor_id, group[i].seq, res.outcome, res.track});
  }
}

void Engine::update_manoeuvre(TrackRecord& r, double d2, Timestamp t) {
  const ManoeuvreConfig& mc = config_.manoeuvre;
  if (mc.accel_psd_factor == 1.0) {
    return;
  }
  r.nis_average = mc.fading * r.nis_average + d2;
  const bool start = !r.manoeuvring && r.nis_average > manoeuvre_on_;
  const bool end = r.manoeuvring && r.nis_average < manoeuvre_off_;
  if (!start && !end) {
    return;
  }
  r.manoeuvring = start;
  (void)r.filter.set_accel_psd(start ? config_.accel_psd * mc.accel_psd_factor : config_.accel_psd);
  TraceEvent e;
  e.kind = start ? TraceEventKind::ManoeuvreStart : TraceEventKind::ManoeuvreEnd;
  e.time = t;
  e.distance2 = r.nis_average;
  e.gate = start ? manoeuvre_on_ : manoeuvre_off_;
  store_.record(r, e);
}

TrackEstimate Engine::estimate_of(const TrackRecord& r) const {
  TrackEstimate e;
  e.id = r.id;
  e.status = r.status;
  e.time = r.time;
  e.last_update = r.time;
  e.created = r.created;
  e.state = r.filter.state();
  e.covariance = r.filter.covariance();
  e.updates = r.updates;
  e.last_sensor = r.last_sensor;
  e.manoeuvring = r.manoeuvring;
  e.trace_total = r.recent.pushed();
  e.sensors.reserve(r.sensors.size());
  for (const store::SensorContribution& c : r.sensors) {
    e.sensors.push_back(c.sensor);
  }
  return e;
}

std::vector<TrackEstimate> Engine::tracks() const {
  std::vector<TrackEstimate> out;
  out.reserve(store_.size());
  for (const TrackRecord& r : store_.live()) {
    out.push_back(estimate_of(r));
  }
  return out;
}

std::optional<TrackEstimate> Engine::track(TrackId id) const {
  const TrackRecord* r = store_.find(id);
  if (r == nullptr) {
    return std::nullopt;
  }
  return estimate_of(*r);
}

std::optional<TrackEstimate> Engine::track_at(TrackId id, Timestamp t,
                                              std::size_t trace_events) const {
  const TrackRecord* r = store_.find(id);
  if (r == nullptr || t < r->time) {
    return std::nullopt;
  }
  estimation::ConstantVelocityFilter f = r->filter;
  if (f.predict(seconds_between(r->time, t)) != FilterStatus::Ok) {
    return std::nullopt;
  }
  TrackEstimate e = estimate_of(*r);
  e.time = t;
  e.state = f.state();
  e.covariance = f.covariance();
  const std::size_t kept = r->recent.size();
  const std::size_t n = std::min(trace_events, kept);
  e.trace.reserve(n);
  for (std::size_t i = kept - n; i < kept; ++i) {
    e.trace.push_back(r->recent.at(i));
  }
  return e;
}

std::vector<TrackEstimate> Engine::tracks_at(Timestamp t, std::size_t trace_events) const {
  std::vector<TrackEstimate> out;
  out.reserve(store_.size());
  for (const TrackRecord& r : store_.live()) {
    if (auto e = track_at(r.id, t, trace_events)) {
      out.push_back(std::move(*e));
    }
  }
  return out;
}

}  // namespace sifted
