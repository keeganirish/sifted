#include "sifted/eval/scenario.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include "sifted/association/assignment.hpp"
#include "sifted/association/gating.hpp"
#include "sifted/net/ingest.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::eval {
namespace {

[[noreturn]] void fail(const std::string& what) { throw std::runtime_error(what); }

std::string read_file(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.good()) {
    fail("cannot open " + path);
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::vector<std::uint8_t> from_hex(const std::string& hex) {
  std::vector<std::uint8_t> out;
  for (std::size_t i = 0; i + 1 < hex.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
  }
  return out;
}

template <class T>
T parse(std::string_view s) {
  T v{};
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (ec != std::errc{} || ptr != s.data() + s.size()) {
    fail("bad number: " + std::string(s));
  }
  return v;
}

}  // namespace

std::vector<std::uint8_t> encode_frame(const Measurement& m) { return wire::encode(m); }

bool is_scenario_dir(const std::string& dir) {
  for (const char* f : {"/sensors.cfg", "/frames.hex", "/truth.csv"}) {
    if (!std::ifstream(dir + f, std::ios::binary).good()) {
      return false;
    }
  }
  return true;
}

Scenario load_scenario_dir(const std::string& path) {
  const std::string dir = path + "/";
  Scenario sc;
  const auto table = parse_sensor_table(read_file(dir + "sensors.cfg"));
  if (!table.ok()) {
    fail(dir + "sensors.cfg: " + std::string(to_string(table.error())));
  }
  sc.sensors = table.value();
  std::istringstream frames(read_file(dir + "frames.hex"));
  for (std::string line; frames >> line;) {
    const auto bytes = from_hex(line);
    const auto m = wire::decode_measurement(bytes);  // the real C++ decoder
    if (!m.ok()) {
      fail(dir + "frames.hex: " + std::string(wire::to_string(m.error())) + ": " + line);
    }
    sc.measurements.push_back(m.value());
  }
  std::istringstream truth(read_file(dir + "truth.csv"));
  std::string line;
  std::getline(truth, line);
  // Layout 2 (multi-target); layout 1 (single target, M1) reads as target 1,
  // always covered.
  const bool v1 = line == "time_ns,x_m,y_m,vx_mps,vy_mps";
  if (!v1 && line != "time_ns,x_m,y_m,vx_mps,vy_mps,target_id,covered") {
    fail(dir + "truth.csv: unknown header: " + line);
  }
  while (std::getline(truth, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    std::vector<std::string_view> f;
    std::string_view rest = line;
    for (std::size_t c; (c = rest.find(',')) != std::string_view::npos; rest.remove_prefix(c + 1)) {
      f.push_back(rest.substr(0, c));
    }
    f.push_back(rest);
    if (v1 && f.size() == 5) {
      sc.truth.push_back({parse<std::int64_t>(f[0]), parse<double>(f[1]), parse<double>(f[2]), 1,
                          true, parse<double>(f[3]), parse<double>(f[4])});
      continue;
    }
    if (f.size() != 7) {
      fail(dir + "truth.csv: expected 7 fields: " + line);
    }
    sc.truth.push_back({parse<std::int64_t>(f[0]), parse<double>(f[1]), parse<double>(f[2]),
                        parse<int>(f[5]), parse<int>(f[6]) != 0, parse<double>(f[3]),
                        parse<double>(f[4])});
  }
  return sc;
}

Scorer::Scorer(const std::vector<TruthRow>& truth, double match_distance_m,
               const GospaParams& gospa)
    : gospa_(gospa), truth_(&truth), match_m_(match_distance_m) {
  if (!is_valid(gospa)) {
    throw std::invalid_argument("invalid GOSPA parameters");
  }
  s_.gospa_params = gospa;
  for (std::size_t r = 0; r < truth.size(); ++r) {
    if (times_.empty() || truth[r].time_ns != times_.back()) {
      times_.push_back(truth[r].time_ns);
      first_row_.push_back(r);
    }
  }
}

namespace {
bool is_reported(association::TrackStatus s) {
  return s == association::TrackStatus::Confirmed || s == association::TrackStatus::Coasting;
}
}  // namespace

void Scorer::score(std::size_t i, const Engine& engine) {
  std::vector<TrackEstimate> reported;
  for (TrackEstimate& e : engine.tracks_at(Timestamp{times_[i]})) {
    if (is_reported(e.status)) {
      reported.push_back(std::move(e));
    }
  }
  score_reported(i, std::move(reported), &engine, true);
}

void Scorer::score(std::size_t i, std::vector<TrackEstimate> tracks, bool covariance) {
  std::erase_if(tracks, [](const TrackEstimate& e) { return !is_reported(e.status); });
  score_reported(i, std::move(tracks), nullptr, covariance);
}

void Scorer::score_reported(std::size_t i, std::vector<TrackEstimate> reported,
                            const Engine* engine, bool covariance) {
  const std::int64_t t = times_[i];
  if (engine == nullptr) {
    s_.engine_metrics = false;
  }
  if (!covariance) {
    s_.covariance = false;
  }
  std::vector<TruthRow> now;
  for (std::size_t row = first_row_[i]; row < truth_->size() && (*truth_)[row].time_ns == t;
       ++row) {
    now.push_back((*truth_)[row]);
  }

  auto distance = [&](std::size_t a, std::size_t j) {
    return std::hypot(reported[j].state(0) - now[a].x, reported[j].state(1) - now[a].y);
  };
  std::vector<int> match(now.size(), association::kUnassigned);
  std::vector<char> taken(reported.size(), 0);
  for (std::size_t a = 0; a < now.size(); ++a) {  // CLEAR-MOT: keep valid correspondences
    const auto it = last_match_.find(now[a].target);
    if (it == last_match_.end()) {
      continue;
    }
    for (std::size_t j = 0; j < reported.size(); ++j) {
      if (reported[j].id == it->second && taken[j] == 0 && distance(a, j) <= match_m_) {
        match[a] = static_cast<int>(j);
        taken[j] = 1;
      }
    }
  }
  std::vector<std::size_t> free_rows;
  std::vector<std::size_t> free_cols;
  for (std::size_t a = 0; a < now.size(); ++a) {
    if (match[a] == association::kUnassigned) {
      free_rows.push_back(a);
    }
  }
  for (std::size_t j = 0; j < reported.size(); ++j) {
    if (taken[j] == 0) {
      free_cols.push_back(j);
    }
  }
  std::vector<double> cost(free_rows.size() * free_cols.size(), association::kForbidden);
  for (std::size_t a = 0; a < free_rows.size(); ++a) {
    for (std::size_t b = 0; b < free_cols.size(); ++b) {
      const double d = distance(free_rows[a], free_cols[b]);
      if (d <= match_m_) {
        cost[a * free_cols.size() + b] = d;
      }
    }
  }
  const std::vector<int> rest =
      association::solve_assignment(cost, free_rows.size(), free_cols.size(), match_m_);
  for (std::size_t a = 0; a < free_rows.size(); ++a) {
    if (rest[a] != association::kUnassigned) {
      match[free_rows[a]] = static_cast<int>(free_cols[static_cast<std::size_t>(rest[a])]);
    }
  }

  InstantRecord rec;
  rec.time_ns = t;
  std::vector<char> matched_track(reported.size(), 0);
  std::vector<char> nis_scored(reported.size(), 0);  // matched to a target past burn-in
  std::vector<int> scored_target(reported.size(), 0);
  std::vector<char> gospa_truth(now.size(), 0);  // in GOSPA's truth set at t
  for (std::size_t a = 0; a < now.size(); ++a) {
    const int tgt = now[a].target;
    rec.covered[tgt] = now[a].covered;
    rec.match[tgt] = 0;
    if (match[a] != association::kUnassigned) {
      const TrackEstimate& e = reported[static_cast<std::size_t>(match[a])];
      matched_track[static_cast<std::size_t>(match[a])] = 1;
      rec.match[tgt] = e.id;
      rec.updated[tgt] = e.last_update.ns;
      if (auto it = last_match_.find(tgt); it != last_match_.end() && it->second != e.id) {
        ++s_.id_switches;
      }
      last_match_[tgt] = e.id;
    }
    if (!now[a].covered) {
      continue;  // out of coverage: may be matched, never missed or scored
    }
    first_covered_.emplace(tgt, t);
    const double age = static_cast<double>(t - first_covered_[tgt]) * 1e-9;
    if (age >= kMissBurnInS) {
      ++s_.eligible;
      s_.missed += match[a] == association::kUnassigned ? 1U : 0U;
      gospa_truth[a] = 1;
    }
    if (age >= kRmseBurnInS) {
      ++s_.rmse_eligible[tgt];
    }
    if (match[a] != association::kUnassigned && age >= kRmseBurnInS) {
      const TrackEstimate& e = reported[static_cast<std::size_t>(match[a])];
      sq_[tgt] += std::pow(e.state(0) - now[a].x, 2) + std::pow(e.state(1) - now[a].y, 2);
      vel_sq_[tgt] += std::pow(e.state(2) - now[a].vx, 2) + std::pow(e.state(3) - now[a].vy, 2);
      ++s_.rmse_n[tgt];
      nis_scored[static_cast<std::size_t>(match[a])] = 1;
      scored_target[static_cast<std::size_t>(match[a])] = tgt;
      if (!covariance) {
        continue;
      }

      // Consistency: same samples as the RMSE.
      const estimation::StateVector err =
          e.state - estimation::StateVector(now[a].x, now[a].y, now[a].vx, now[a].vy);
      const double full = nees(err, e.covariance);
      const double pos = nees(err.head<2>(), e.covariance.topLeftCorner<2, 2>());
      if (!std::isfinite(full) || !std::isfinite(pos)) {
        fail("non-finite NEES: track " + std::to_string(e.id) + " at " + std::to_string(t));
      }
      const bool updated = e.last_update.ns == t;
      for (NeesByRegime* agg : {&s_.nees, &s_.nees_by_target[tgt]}) {
        NeesStats& st = updated ? agg->updated : agg->extrapolated;
        st.full.add(full, static_cast<std::size_t>(err.size()));
        st.position.add(pos, 2);
      }
      rec.nees[tgt] = full;
    }
  }
  // Tracks not matched at a scored target-instant still have their trace read,
  // so their earlier updates are not credited to a later match.
  if (engine != nullptr) {
    for (std::size_t j = 0; j < reported.size(); ++j) {
      const bool scored = nis_scored[j] != 0;
      collect_nis(*engine, reported[j].id, scored, scored ? &rec.nis[scored_target[j]] : nullptr);
    }
  }
  // GOSPA (header comment): truth = targets eligible for "missed"; estimates
  // = reported tracks not matched to a present target outside that set.
  if (!times_.empty() && static_cast<double>(t - times_.front()) * 1e-9 >= kMissBurnInS) {
    std::vector<char> dont_care(reported.size(), 0);
    std::vector<Point2> xs;
    for (std::size_t a = 0; a < now.size(); ++a) {
      if (gospa_truth[a] != 0) {
        xs.push_back({now[a].x, now[a].y});
      } else if (match[a] != association::kUnassigned) {
        dont_care[static_cast<std::size_t>(match[a])] = 1;
      }
    }
    std::vector<Point2> ys;
    for (std::size_t j = 0; j < reported.size(); ++j) {
      if (dont_care[j] == 0) {
        ys.push_back({reported[j].state(0), reported[j].state(1)});
      }
    }
    s_.gospa.add(gospa(xs, ys, gospa_), gospa_.p);
  }
  for (std::size_t j = 0; j < reported.size(); ++j) {
    auto& c = track_counts_[reported[j].id];
    ++c.first;
    c.second += matched_track[j] != 0 ? 1U : 0U;
    s_.trace.push_back(reported[j]);
  }
  s_.instants.push_back(std::move(rec));
}

void Scorer::collect_nis(const Engine& engine, TrackId track, bool score, ChiSquareStat* instant) {
  if (engine.store().find(track) == nullptr) {
    return;
  }
  const std::vector<store::TraceEvent> events =
      new_associated_events(engine, track, nis_read_[track], s_.nis_lost);
  if (!score) {
    return;
  }
  for (const store::TraceEvent& e : events) {
    const auto idx = engine.sensors().index_of(e.sensor);
    if (!idx) {
      fail("trace names unknown sensor " + std::to_string(e.sensor));
    }
    auto [dof_it, inserted] = dof_of_gate_.try_emplace(e.gate, 0);
    if (inserted) {
      dof_it->second = gate_dof(engine, e.gate);
    }
    // The NIS under the track estimate (TraceEvent::nis; the gating d^2 for the
    // constant-velocity filter).
    const double nis = std::isnan(e.nis) ? e.distance2 : e.nis;
    s_.nis[engine.sensors().sensors()[*idx].kind].add(nis, dof_it->second);
    if (instant != nullptr) {
      instant->add(nis, dof_it->second);
    }
  }
}

std::vector<store::TraceEvent> new_associated_events(const Engine& engine, TrackId track,
                                                     std::uint64_t& read, std::size_t& lost) {
  std::vector<store::TraceEvent> out;
  const store::TrackRecord* r = engine.store().find(track);
  if (r == nullptr) {
    return out;
  }
  const std::uint64_t total =
      r->totals[static_cast<std::size_t>(store::TraceEventKind::Associated)];
  const std::uint64_t fresh = total - read;
  read = total;
  // The newest `fresh` Associated events, newest first.
  for (std::size_t i = r->recent.size(); i-- > 0 && out.size() < fresh;) {
    if (r->recent.at(i).kind == store::TraceEventKind::Associated) {
      out.push_back(r->recent.at(i));
    }
  }
  lost += fresh - out.size();
  std::reverse(out.begin(), out.end());
  return out;
}

std::size_t gate_dof(const Engine& engine, double gate) {
  const double p = engine.config().association.gate_probability;
  for (std::size_t d = 1; d <= association::kMaxChiSquareDof; ++d) {
    if (association::chi_square_quantile(d, p) == gate) {
      return d;
    }
  }
  fail("no dof gives gate " + std::to_string(gate));
}

Score Scorer::finish(const Engine& engine) {
  Score s = finish_common();
  s.tracks_started = engine.tracks_started();
  s.track_limit = engine.count(MeasurementOutcome::TrackLimit);
  s.out_of_order = engine.count(MeasurementOutcome::OutOfOrder);
  s.measurements = engine.measurements_processed();
  return s;
}

Score Scorer::finish() {
  Score s = finish_common();
  s.engine_metrics = false;
  return s;
}

Score Scorer::finish_common() const {
  Score s = s_;
  for (const auto& [tgt, sum] : sq_) {
    s.rmse[tgt] = std::sqrt(sum / static_cast<double>(s.rmse_n[tgt]));
    s.mean_rmse += s.rmse[tgt] / static_cast<double>(sq_.size());
    s.max_rmse = std::max(s.max_rmse, s.rmse[tgt]);
  }
  for (const auto& [tgt, sum] : vel_sq_) {
    s.velocity_rmse[tgt] = std::sqrt(sum / static_cast<double>(s.rmse_n[tgt]));
    s.mean_velocity_rmse += s.velocity_rmse[tgt] / static_cast<double>(vel_sq_.size());
  }
  for (const auto& [id, c] : track_counts_) {
    if (2 * c.second < c.first) {
      ++s.false_tracks;
    }
  }
  s.reported_tracks = track_counts_.size();
  return s;
}

Score run_engine(const Scenario& sc, const std::vector<Measurement>& measurements,
                 const EngineConfig& cfg, double match_distance_m, const GospaParams& gospa,
                 const InstantObserver& on_instant) {
  Engine engine(sc.sensors, cfg);
  Scorer scorer(sc.truth, match_distance_m, gospa);
  std::vector<MeasurementResult> results;
  std::size_t next = 0;
  for (std::size_t i = 0; i < scorer.instants().size(); ++i) {
    const std::int64_t t = scorer.instants()[i];
    std::size_t end = next;
    while (end < measurements.size() && measurements[end].time.ns <= t) {
      ++end;
    }
    engine.on_scan(std::span<const Measurement>(measurements).subspan(next, end - next), results);
    next = end;
    scorer.score(i, engine);
    if (on_instant) {
      on_instant(t, engine);
    }
  }
  Score s = scorer.finish(engine);
  for (std::size_t i = 0; i < results.size() && i < measurements.size(); ++i) {
    if (measurements[i].kind == SensorKind::BearingOnly) {
      ++s.bearings;
      s.bearings_associated += results[i].outcome == MeasurementOutcome::Updated ? 1U : 0U;
    }
  }
  return s;
}

Score run_engine(const Scenario& sc, const EngineConfig& cfg, double match_distance_m) {
  return run_engine(sc, sc.measurements, cfg, match_distance_m);
}

std::vector<net::SentFrame> sent_frames(const std::vector<Measurement>& measurements) {
  std::vector<net::SentFrame> out;
  out.reserve(measurements.size());
  for (const Measurement& m : measurements) {
    out.push_back({m.time, wire::encode(m)});
  }
  return out;
}

std::vector<net::ArrivingFrame> clean_arrivals(const std::vector<net::SentFrame>& frames) {
  std::vector<net::ArrivingFrame> out;
  out.reserve(frames.size());
  for (std::size_t i = 0; i < frames.size(); ++i) {
    out.push_back({frames[i].send, i, false, frames[i].bytes});
  }
  return out;
}

PipelineRun run_pipeline(const Scenario& sc, const std::vector<net::ArrivingFrame>& arrivals,
                         const net::PipelineConfig& cfg, std::int64_t tick_ns,
                         double match_distance_m, const GospaParams& gospa) {
  PipelineRun run;
  std::size_t bearings = 0;
  std::size_t bearings_associated = 0;
  const SensorTable& table = sc.sensors;
  net::Pipeline pipeline(sc.sensors, cfg, [&](std::string_view line) {
    run.output.append(line);
    // m <recv> <sensor> <seq> <time> <outcome> <track>
    if (line.size() > 2 && line[0] == 'm') {
      std::istringstream ls{std::string(line.substr(2))};
      std::string recv;
      std::string outcome;
      unsigned sensor = 0;
      ls >> recv >> sensor >> recv >> recv >> outcome;
      const auto idx = table.index_of(static_cast<SensorId>(sensor));
      if (idx && table.sensors()[*idx].kind == SensorKind::BearingOnly) {
        ++bearings;
        bearings_associated += outcome == "updated" ? 1U : 0U;
      }
    }
  });
  Scorer scorer(sc.truth, match_distance_m, gospa);
  std::optional<Timestamp> last;  // time of the last event consumed
  std::size_t next = 0;
  // Consume every event (frame or quiet tick) at or before `until`.
  auto advance = [&](std::int64_t until) {
    while (true) {
      const bool frame_due = next < arrivals.size() && arrivals[next].arrival.ns <= until;
      // The daemon ticks when its queue stays empty for tick_ns with work pending.
      if (last && pipeline.has_pending()) {
        const std::int64_t tick_at = last->ns + tick_ns;
        const bool tick_first = !frame_due || tick_at < arrivals[next].arrival.ns;
        if (tick_at <= until && tick_first) {
          pipeline.on_tick(Timestamp{tick_at});
          last = Timestamp{tick_at};
          ++run.ticks;
          continue;
        }
      }
      if (!frame_due) {
        return;
      }
      const net::ArrivingFrame& a = arrivals[next++];
      pipeline.on_frame(a.arrival, a.bytes);
      last = a.arrival;
    }
  };
  for (std::size_t i = 0; i < scorer.instants().size(); ++i) {
    advance(scorer.instants()[i]);
    scorer.score(i, pipeline.engine());
  }
  advance(INT64_MAX - tick_ns);
  if (last) {
    pipeline.finish(*last);
  }
  run.score = scorer.finish(pipeline.engine());
  run.score.bearings = bearings;
  run.score.bearings_associated = bearings_associated;
  run.duplicates = pipeline.duplicates();
  run.future_rejected = pipeline.future_rejected();
  run.reorder_late = pipeline.reorder_late();
  run.reorder_forced = pipeline.reorder_forced();
  run.held_high_water = pipeline.held_high_water();
  return run;
}

Recovery recovery_after(const Score& s, std::int64_t outage_start_ns, std::int64_t outage_end_ns) {
  Recovery r;
  const InstantRecord* before = nullptr;
  for (const InstantRecord& rec : s.instants) {
    if (rec.time_ns < outage_start_ns) {
      before = &rec;
    }
  }
  if (before == nullptr) {
    return r;
  }
  std::vector<std::pair<int, TrackId>> tracked;  // target, track before the outage
  for (const auto& [tgt, trk] : before->match) {
    if (trk != 0 && before->covered.at(tgt)) {
      tracked.emplace_back(tgt, trk);
      r.last_update_before[tgt] = before->updated.at(tgt);
    }
  }
  for (const InstantRecord& rec : s.instants) {
    if (rec.time_ns < outage_end_ns) {
      continue;
    }
    // Only targets still present and covered after the outage count.
    std::vector<std::pair<int, TrackId>> still;
    for (const auto& [tgt, trk] : tracked) {
      const auto c = rec.covered.find(tgt);
      if (c != rec.covered.end() && c->second) {
        still.emplace_back(tgt, trk);
      }
    }
    r.targets = still.size();
    const bool all = std::all_of(still.begin(), still.end(), [&](const auto& p) {
      return rec.match.at(p.first) != 0 && rec.updated.at(p.first) >= outage_end_ns;
    });
    if (all) {
      r.recovery_s = static_cast<double>(rec.time_ns - outage_end_ns) * 1e-9;
      for (const auto& [tgt, trk] : still) {
        const bool same = rec.match.at(tgt) == trk;
        (same ? r.survived : r.reinitiated) += 1;
        r.target_survived[tgt] = same;
      }
      return r;
    }
  }
  return r;
}

double velocity_change(const Scenario& sc, int target, std::int64_t from_ns, std::int64_t to_ns) {
  const TruthRow* first = nullptr;
  double worst = 0.0;
  for (const TruthRow& r : sc.truth) {
    if (r.target != target || r.time_ns < from_ns || r.time_ns > to_ns) {
      continue;
    }
    if (first == nullptr) {
      first = &r;
    }
    worst = std::max(worst, std::hypot(r.vx - first->vx, r.vy - first->vy));
  }
  return worst;
}

}  // namespace sifted::eval
