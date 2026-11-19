#include "sifted/eval/realdata.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <fstream>
#include <set>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <type_traits>

#include "sifted/eval/scenario.hpp"
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

template <class T>
T parse(std::string_view s, const std::string& where) {
  T v{};
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (s.empty() || ec != std::errc{} || ptr != s.data() + s.size()) {
    fail(where + ": bad number '" + std::string(s) + "'");
  }
  if constexpr (std::is_floating_point_v<T>) {
    if (!std::isfinite(v)) {
      fail(where + ": non-finite number");
    }
  }
  return v;
}

std::vector<std::string_view> split(std::string_view line, char sep) {
  std::vector<std::string_view> f;
  for (std::size_t c; (c = line.find(sep)) != std::string_view::npos; line.remove_prefix(c + 1)) {
    f.push_back(line.substr(0, c));
  }
  f.push_back(line);
  return f;
}

// Lines of a CSV file after its header, which must be `header`.
std::vector<std::string> csv_rows(const std::string& path, std::string_view header) {
  std::istringstream in(read_file(path));
  std::vector<std::string> rows;
  std::string line;
  if (!std::getline(in, line) || line != header) {
    fail(path + ": expected header " + std::string(header));
  }
  while (std::getline(in, line)) {
    if (!line.empty()) {
      rows.push_back(line);
    }
  }
  return rows;
}

std::vector<std::uint8_t> from_hex(const std::string& hex, const std::string& where) {
  if (hex.size() % 2 != 0) {
    fail(where + ": odd-length hex");
  }
  std::vector<std::uint8_t> out;
  for (std::size_t i = 0; i < hex.size(); i += 2) {
    std::uint8_t b = 0;
    const char* p = hex.data() + i;
    const auto [end, ec] = std::from_chars(p, p + 2, b, 16);
    if (ec != std::errc{} || end != p + 2) {
      fail(where + ": bad hex");
    }
    out.push_back(b);
  }
  return out;
}

// The local east and north unit vectors at (lat, lon), in ECEF.
std::pair<Eigen::Vector3d, Eigen::Vector3d> east_north(double lat_deg, double lon_deg) {
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double sl = std::sin(lat_deg * kDegToRad);
  const double cl = std::cos(lat_deg * kDegToRad);
  const double so = std::sin(lon_deg * kDegToRad);
  const double co = std::cos(lon_deg * kDegToRad);
  return {Eigen::Vector3d(-so, co, 0.0), Eigen::Vector3d(-sl * co, -sl * so, cl)};
}

Eigen::Vector2d local(const estimation::GeoOrigin& origin, const estimation::Geodetic& g) {
  return estimation::geodetic_to_enu(origin, g).head<2>();
}

estimation::Geodetic position_of(const Measurement& m) { return {m.z[0], m.z[1], m.z[2]}; }

}  // namespace

const char* to_string(Estimator e) noexcept {
  switch (e) {
    case Estimator::Tracker:
      return "tracker";
    case Estimator::Hold:
      return "hold";
    case Estimator::DeadReckoning:
      return "dead_reckoning";
  }
  return "unknown";
}

bool is_holdout_dir(const std::string& dir) {
  for (const char* f :
       {"/sensors.cfg", "/origin.txt", "/frames.hex", "/feed_key.csv", "/withheld.csv"}) {
    if (!std::ifstream(dir + f, std::ios::binary).good()) {
      return false;
    }
  }
  return true;
}

HoldoutRun load_holdout_dir(const std::string& path) {
  const std::string dir = path + "/";
  HoldoutRun run;
  const auto table = parse_sensor_table(read_file(dir + "sensors.cfg"));
  if (!table.ok()) {
    fail(dir + "sensors.cfg: " + std::string(to_string(table.error())));
  }
  run.sensors = table.value();

  std::istringstream origin(read_file(dir + "origin.txt"));
  std::string lat;
  std::string lon;
  std::string alt;
  std::string extra;
  if (!(origin >> lat >> lon >> alt) || (origin >> extra)) {
    fail(dir + "origin.txt: expected 'lat lon alt'");
  }
  run.origin = {parse<double>(lat, dir + "origin.txt"), parse<double>(lon, dir + "origin.txt"),
                parse<double>(alt, dir + "origin.txt")};
  if (!estimation::is_valid(run.origin)) {
    fail(dir + "origin.txt: origin out of range");
  }

  std::istringstream frames(read_file(dir + "frames.hex"));
  for (std::string line; frames >> line;) {
    const auto m = wire::decode_measurement(from_hex(line, dir + "frames.hex"));
    if (!m.ok()) {
      fail(dir + "frames.hex: " + std::string(wire::to_string(m.error())) + ": " + line);
    }
    if (!run.measurements.empty() && m.value().time < run.measurements.back().time) {
      fail(dir + "frames.hex: frames out of time order");
    }
    run.measurements.push_back(m.value());
  }

  const std::string key_path = dir + "feed_key.csv";
  for (const std::string& line : csv_rows(key_path, "frame,aircraft,speed_mps,track_deg")) {
    const auto f = split(line, ',');
    if (f.size() != 4) {
      fail(key_path + ": expected 4 fields: " + line);
    }
    if (parse<std::size_t>(f[0], key_path) != run.key.size()) {
      fail(key_path + ": frames must be numbered 0, 1, 2, ...: " + line);
    }
    FeedKey k;
    k.aircraft = parse<int>(f[1], key_path);
    if (k.aircraft < 1) {
      fail(key_path + ": aircraft labels start at 1: " + line);
    }
    if (f[2].empty() != f[3].empty()) {
      fail(key_path + ": speed and track must both be given or both be empty: " + line);
    }
    if (!f[2].empty()) {
      k.speed_mps = parse<double>(f[2], key_path);
      k.track_deg = parse<double>(f[3], key_path);
    }
    run.key.push_back(k);
  }
  if (run.key.size() != run.measurements.size()) {
    fail(key_path + ": one row per frame expected");
  }

  const std::string w_path = dir + "withheld.csv";
  for (const std::string& line : csv_rows(w_path, "time_ns,aircraft,lat,lon,alt_m")) {
    const auto f = split(line, ',');
    if (f.size() != 5) {
      fail(w_path + ": expected 5 fields: " + line);
    }
    WithheldReport w{
        parse<std::int64_t>(f[0], w_path),
        parse<int>(f[1], w_path),
        {parse<double>(f[2], w_path), parse<double>(f[3], w_path), parse<double>(f[4], w_path)}};
    if (w.aircraft < 1 || std::abs(w.position.lat_deg) > 90.0 ||
        std::abs(w.position.lon_deg) > 180.0) {
      fail(w_path + ": out of range: " + line);
    }
    if (!run.withheld.empty() && w.time_ns < run.withheld.back().time_ns) {
      fail(w_path + ": rows out of time order");
    }
    run.withheld.push_back(w);
  }
  return run;
}

void HoldoutScore::merge(const HoldoutScore& o) {
  for (std::size_t i = 0; i < errors.size(); ++i) {
    errors[i].insert(errors[i].end(), o.errors[i].begin(), o.errors[i].end());
  }
  withheld += o.withheld;
  no_prior_report += o.no_prior_report;
  no_track += o.no_track;
  no_velocity += o.no_velocity;
  nis.merge(o.nis);
  nis_lost += o.nis_lost;
  tracks_started += o.tracks_started;
  confirmed_tracks += o.confirmed_tracks;
  fragments += o.fragments;
  never_confirmed += o.never_confirmed;
  swaps += o.swaps;
  mixed_tracks += o.mixed_tracks;
  inconsistent_tracks += o.inconsistent_tracks;
  max_range_m = std::max(max_range_m, o.max_range_m);
  for (const auto& [k, v] : o.outcomes) {
    outcomes[k] += v;
  }
  for (const auto& [a, c] : o.aircraft) {
    AircraftContinuity& mine = aircraft[a];
    mine.fed += c.fed;
    mine.confirmed_tracks += c.confirmed_tracks;
    mine.coverage_eligible += c.coverage_eligible;
    mine.covered += c.covered;
  }
}

std::size_t HoldoutScore::coverage_eligible() const {
  std::size_t n = 0;
  for (const auto& [a, c] : aircraft) {
    n += c.coverage_eligible;
  }
  return n;
}

std::size_t HoldoutScore::covered() const {
  std::size_t n = 0;
  for (const auto& [a, c] : aircraft) {
    n += c.covered;
  }
  return n;
}

HoldoutScore score_holdout(const HoldoutRun& run, EngineConfig cfg) {
  cfg.origin = run.origin;
  Engine engine(run.sensors, cfg);
  HoldoutScore s;
  const std::vector<Measurement>& ms = run.measurements;
  const auto [o_east, o_north] = east_north(run.origin.lat_deg, run.origin.lon_deg);

  std::vector<MeasurementResult> results;
  std::map<int, std::size_t> last_fed;         // aircraft -> index of its last fed report
  std::map<int, TrackId> last_track;           // aircraft -> track its last used report went to
  std::map<TrackId, std::vector<int>> labels;  // aircraft of each update, in order
  std::set<TrackId> confirmed;
  std::map<int, std::set<TrackId>> aircraft_confirmed;
  std::map<TrackId, std::uint64_t> nis_read;
  std::map<TrackId, std::uint64_t> nis_seen;
  std::map<double, std::size_t> dof;
  std::size_t next = 0;

  // One group exactly as Engine::on_scan forms them (a maximal run of
  // measurements with one sensor and one timestamp), so track status and the
  // trace are read after every group, never only after a long batch.
  auto feed_group = [&](std::size_t end) {
    const std::size_t first = results.size();
    engine.on_scan(std::span<const Measurement>(ms).subspan(next, end - next), results);
    std::vector<TrackId> touched;
    for (std::size_t i = next; i < end; ++i) {
      const MeasurementResult& r = results[first + (i - next)];
      ++s.outcomes[r.outcome];
      const int a = run.key[i].aircraft;
      AircraftContinuity& ac = s.aircraft[a];
      const std::size_t before = ac.fed++;
      last_fed[a] = i;
      const double range = local(run.origin, position_of(ms[i])).norm();
      s.max_range_m = std::max(s.max_range_m, static_cast<std::size_t>(range));
      const bool used =
          r.outcome == MeasurementOutcome::Updated || r.outcome == MeasurementOutcome::Initialized;
      bool now_confirmed = false;
      if (used) {
        last_track[a] = r.track;
        labels[r.track].push_back(a);
        touched.push_back(r.track);
        const auto est = engine.track(r.track);
        now_confirmed = est && est->status == association::TrackStatus::Confirmed;
        if (now_confirmed) {
          confirmed.insert(r.track);
          aircraft_confirmed[a].insert(r.track);
        }
      }
      if (before >= engine.config().lifecycle.confirm_hits) {
        ++ac.coverage_eligible;
        ac.covered += now_confirmed ? 1U : 0U;
      }
    }
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    for (const TrackId tr : touched) {
      for (const store::TraceEvent& e :
           new_associated_events(engine, tr, nis_read[tr], s.nis_lost)) {
        if (++nis_seen[tr] <= kNisBurnInUpdates) {
          continue;
        }
        auto [it, inserted] = dof.try_emplace(e.gate, 0);
        if (inserted) {
          it->second = gate_dof(engine, e.gate);
        }
        s.nis.add(e.distance2, it->second);
      }
    }
    next = end;
  };
  auto feed_until = [&](std::int64_t t) {
    while (next < ms.size() && ms[next].time.ns <= t) {
      std::size_t end = next + 1;
      while (end < ms.size() && ms[end].time == ms[next].time &&
             ms[end].sensor_id == ms[next].sensor_id) {
        ++end;
      }
      feed_group(end);
    }
  };

  for (const WithheldReport& w : run.withheld) {
    feed_until(w.time_ns);
    ++s.withheld;
    const auto lf = last_fed.find(w.aircraft);
    if (lf == last_fed.end()) {
      ++s.no_prior_report;
      continue;
    }
    const Measurement& m = ms[lf->second];
    const double horizon = static_cast<double>(w.time_ns - m.time.ns) * 1e-9;
    const Eigen::Vector2d truth = local(run.origin, w.position);
    const Eigen::Vector2d last = local(run.origin, position_of(m));
    auto add = [&](Estimator e, const Eigen::Vector2d& p) {
      s.errors[static_cast<std::size_t>(e)].push_back({horizon, (p - truth).norm()});
    };
    add(Estimator::Hold, last);

    const FeedKey& k = run.key[lf->second];
    if (k.speed_mps && k.track_deg) {
      // The reported velocity, rotated from the report's local frame into the run's.
      constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
      const auto [e, n] = east_north(m.z[0], m.z[1]);
      const Eigen::Vector3d v = *k.speed_mps * (std::sin(*k.track_deg * kDegToRad) * e +
                                                std::cos(*k.track_deg * kDegToRad) * n);
      add(Estimator::DeadReckoning,
          last + horizon * Eigen::Vector2d(o_east.dot(v), o_north.dot(v)));
    } else {
      ++s.no_velocity;
    }

    bool estimated = false;
    if (const auto lt = last_track.find(w.aircraft); lt != last_track.end() && engine.time()) {
      for (const TrackEstimate& e : engine.tracks_if_silent_until(Timestamp{w.time_ns})) {
        if (e.id == lt->second && (e.status == association::TrackStatus::Confirmed ||
                                   e.status == association::TrackStatus::Coasting)) {
          add(Estimator::Tracker, Eigen::Vector2d(e.state(0), e.state(1)));
          estimated = true;
        }
      }
    }
    s.no_track += estimated ? 0U : 1U;
  }
  feed_until(INT64_MAX);

  for (const auto& [tr, who] : labels) {
    if (!confirmed.contains(tr)) {
      continue;
    }
    ++s.confirmed_tracks;
    std::map<int, std::size_t> count;
    for (std::size_t i = 0; i < who.size(); ++i) {
      ++count[who[i]];
      s.swaps += i > 0 && who[i] != who[i - 1] ? 1U : 0U;
    }
    s.mixed_tracks += count.size() > 1 ? 1U : 0U;
    std::size_t top = 0;
    for (const auto& [a, n] : count) {
      top = std::max(top, n);
    }
    s.inconsistent_tracks += 2 * top <= who.size() ? 1U : 0U;
  }
  for (auto& [a, c] : s.aircraft) {
    c.confirmed_tracks = aircraft_confirmed[a].size();
    s.fragments += c.confirmed_tracks > 0 ? c.confirmed_tracks - 1 : 0;
    s.never_confirmed += c.confirmed_tracks == 0 ? 1U : 0U;
  }
  s.tracks_started = engine.tracks_started();
  return s;
}

Quantiles quantiles(std::vector<double> v) {
  Quantiles q;
  q.n = v.size();
  if (v.empty()) {
    return q;
  }
  std::sort(v.begin(), v.end());
  auto rank = [&](double p) {
    const auto k = static_cast<std::size_t>(std::ceil(p * static_cast<double>(v.size())));
    return v[std::min(std::max<std::size_t>(k, 1), v.size()) - 1];
  };
  q.median = rank(0.5);
  q.p95 = rank(0.95);
  q.p99 = rank(0.99);
  double sum = 0.0;
  for (const double x : v) {
    sum += x;
  }
  q.mean = sum / static_cast<double>(v.size());
  return q;
}

}  // namespace sifted::eval
