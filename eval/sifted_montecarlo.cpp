// sifted_montecarlo: Monte Carlo evaluation over independent noise
// realisations (docs/evaluation-monte-carlo.md, docs/evaluation-baseline.md,
// ADR 0015).
//
// Reads a manifest written by `python3 -m sifted_sim.montecarlo generate`
// (one row per run: set, condition, scenario, run index, seeds, sweep
// parameters, directory), runs Sifted in process on every run (direct engine
// feed, and for the noise set also through net::Pipeline under a seeded
// network impairment level), scores external trackers' track reports
// (track_report.hpp) with the same Scorer, and writes one page with interval
// estimates over runs (stats.hpp): the run is the independent unit.
//
// Deterministic: the manifest fixes the inputs, the bootstrap is seeded,
// iteration is sorted, numbers are formatted with fixed precision, and no
// clock or path reaches the page, so the same binary on the same
// architecture writes the same bytes. Timing (--timing) goes to a separate
// file because it is not reproducible.

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "sifted/eval/conditions.hpp"
#include "sifted/eval/gospa.hpp"
#include "sifted/eval/scenario.hpp"
#include "sifted/eval/stats.hpp"
#include "sifted/eval/track_report.hpp"
#include "sifted/net/impairment.hpp"
#include "sifted/net/pipeline_config.hpp"

namespace sifted::eval {
namespace {

namespace fs = std::filesystem;

constexpr const char* kUsage =
    "usage: sifted_montecarlo --manifest FILE [options]\n"
    "  --manifest FILE     runs to evaluate (python3 -m sifted_sim.montecarlo generate)\n"
    "  --trackers LIST     in-process Sifted configurations, comma-separated (default sifted):\n"
    "                        sifted             EngineConfig defaults (IMM, ADR 0014)\n"
    "                        sifted-cv          legacy constant-velocity filter with its\n"
    "                                           manoeuvre process-noise switching\n"
    "                        sifted-cv-fixed-q  constant velocity, fixed process noise\n"
    "  --tracks NAME=DIR   an external tracker: DIR/<run dir>.tracks for every run; repeatable\n"
    "  --reference NAME    the tracker the others are compared with (default: the first)\n"
    "  --network LEVEL     impairment level of the noise set's pipeline condition, for\n"
    "                      in-process trackers (default moderate; none: skip)\n"
    "  --window S          reorder window of that condition [s] (default 0.5)\n"
    "  --page montecarlo|baseline  page layout (default montecarlo)\n"
    "  --bootstrap N       bootstrap resamples (default 2000)\n"
    "  --stats-seed N      bootstrap base seed (default 1)\n"
    "  --gospa-c M         GOSPA cut-off [m] (default 100)\n"
    "  --gospa-p P         GOSPA order (default 2)\n"
    "  --format md|csv     page, or one CSV row per (tracker, condition, run) (default md)\n"
    "  --export-tracks DIR write the first in-process tracker's direct-feed track reports\n"
    "  --timing FILE       time each in-process tracker's direct feed (CSV; not reproducible)\n"
    "  --out FILE          write to FILE instead of standard output\n";

// ---- options ------------------------------------------------------------------

struct External {
  std::string name;
  std::string dir;
};

struct Options {
  std::string manifest;
  std::vector<std::string> trackers{"sifted"};
  std::vector<External> externals;
  std::string reference;
  std::string network{"moderate"};
  double window_s = kDaemonReorderWindowS;
  bool baseline_page = false;
  std::size_t bootstrap = kBootstrapResamples;
  std::uint64_t stats_seed = 1;
  GospaParams gospa;
  bool markdown = true;
  std::string export_dir;
  std::string timing;
  std::string out;
  std::vector<std::string> args;  // echoed (paths replaced by their role)
  bool help = false;
};

std::vector<std::string> split(std::string_view s, char sep = ',') {
  std::vector<std::string> out;
  while (true) {
    const std::size_t c = s.find(sep);
    out.emplace_back(s.substr(0, c));
    if (c == std::string_view::npos) {
      return out;
    }
    s.remove_prefix(c + 1);
  }
}

template <class T>
T number(std::string_view s, const char* what) {
  T v{};
  const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (s.empty() || ec != std::errc{} || p != s.data() + s.size()) {
    throw std::invalid_argument(std::string("bad ") + what + ": '" + std::string(s) + "'");
  }
  return v;
}

std::optional<EngineConfig> tracker_config(const std::string& name) {
  EngineConfig c;
  if (name == "sifted") {
    return c;
  }
  c.motion_model = estimation::MotionModel::ConstantVelocity;
  if (name == "sifted-cv") {
    return c;
  }
  if (name == "sifted-cv-fixed-q") {
    c.manoeuvre.accel_psd_factor = 1.0;  // disables the process-noise switching
    return c;
  }
  return std::nullopt;
}

Options parse_args(int argc, char** argv) {
  Options o;
  const std::set<std::string> known = {"--manifest",      "--trackers", "--tracks",  "--reference",
                                       "--network",       "--window",   "--page",    "--bootstrap",
                                       "--stats-seed",    "--gospa-c",  "--gospa-p", "--format",
                                       "--export-tracks", "--timing",   "--out"};
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      o.help = true;
      return o;
    }
    if (!known.contains(a)) {
      throw std::invalid_argument("unknown option " + a);
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value for " + a);
    }
    const std::string v = argv[++i];
    if (a == "--manifest") {
      o.manifest = v;
      o.args.insert(o.args.end(), {a, "MANIFEST"});
      continue;
    }
    if (a == "--out" || a == "--export-tracks" || a == "--timing") {
      (a == "--out" ? o.out : a == "--timing" ? o.timing : o.export_dir) = v;
      continue;  // output locations are not part of what was evaluated
    }
    if (a == "--tracks") {
      const std::size_t eq = v.find('=');
      if (eq == std::string::npos || eq == 0 || eq + 1 == v.size()) {
        throw std::invalid_argument("--tracks needs NAME=DIR");
      }
      o.externals.push_back({v.substr(0, eq), v.substr(eq + 1)});
      o.args.insert(o.args.end(), {a, v.substr(0, eq) + "=DIR"});
      continue;
    }
    o.args.insert(o.args.end(), {a, v});
    if (a == "--trackers") {
      o.trackers = split(v);
      for (const std::string& t : o.trackers) {
        if (!tracker_config(t)) {
          throw std::invalid_argument("unknown tracker " + t);
        }
      }
    } else if (a == "--reference") {
      o.reference = v;
    } else if (a == "--network") {
      if (v != "none") {
        (void)impairment_level(v);  // throws for an unknown level
      }
      o.network = v;
    } else if (a == "--window") {
      o.window_s = number<double>(v, "window");
      if (!(o.window_s >= 0.0 && o.window_s <= 60.0)) {
        throw std::invalid_argument("window out of range");
      }
    } else if (a == "--page") {
      if (v != "montecarlo" && v != "baseline") {
        throw std::invalid_argument("--page must be montecarlo or baseline");
      }
      o.baseline_page = v == "baseline";
    } else if (a == "--bootstrap") {
      o.bootstrap = number<std::size_t>(v, "bootstrap");
      if (o.bootstrap == 0 || o.bootstrap > 100000) {
        throw std::invalid_argument("bootstrap out of range");
      }
    } else if (a == "--stats-seed") {
      o.stats_seed = number<std::uint64_t>(v, "stats seed");
    } else if (a == "--gospa-c") {
      o.gospa.c = number<double>(v, "GOSPA c");
    } else if (a == "--gospa-p") {
      o.gospa.p = number<double>(v, "GOSPA p");
    } else if (a == "--format") {
      if (v != "csv" && v != "md") {
        throw std::invalid_argument("--format must be csv or md");
      }
      o.markdown = v == "md";
    }
  }
  if (o.manifest.empty()) {
    throw std::invalid_argument("--manifest is required");
  }
  if (!is_valid(o.gospa)) {
    throw std::invalid_argument("GOSPA c must be > 0 and p >= 1");
  }
  std::set<std::string> names(o.trackers.begin(), o.trackers.end());
  if (names.size() != o.trackers.size()) {
    throw std::invalid_argument("a tracker is named twice");
  }
  for (const External& e : o.externals) {
    if (!names.insert(e.name).second || tracker_config(e.name)) {
      throw std::invalid_argument("external tracker name clashes: " + e.name);
    }
  }
  if (o.reference.empty()) {
    o.reference = o.trackers.empty() ? o.externals.front().name : o.trackers.front();
  }
  if (!names.contains(o.reference)) {
    throw std::invalid_argument("--reference names no tracker: " + o.reference);
  }
  if (o.trackers.empty() && o.externals.empty()) {
    throw std::invalid_argument("no tracker to evaluate");
  }
  return o;
}

// ---- manifest -------------------------------------------------------------------

struct ManifestRun {
  std::string set;
  std::string condition;
  std::string scenario;
  int run = 0;
  std::uint64_t seed = 0;
  std::uint64_t impairment_seed = 0;
  std::optional<double> p_detect;
  std::optional<double> clutter;
  std::string dir;  // relative to the manifest
};

struct Manifest {
  std::uint64_t base_seed = 0;
  std::vector<ManifestRun> runs;
  fs::path root;
};

bool safe_relative(const std::string& dir) {
  const fs::path p(dir);
  if (dir.empty() || p.is_absolute() || p.has_root_name()) {
    return false;
  }
  return std::none_of(p.begin(), p.end(), [](const fs::path& part) { return part == ".."; });
}

std::optional<double> opt_number(const std::string& s, const char* what) {
  return s.empty() ? std::nullopt : std::optional(number<double>(s, what));
}

Manifest read_manifest(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f.good()) {
    throw std::runtime_error("cannot open manifest " + path);
  }
  Manifest m;
  m.root = fs::path(path).parent_path();
  std::string line;
  std::size_t n = 0;
  std::set<std::string> dirs;
  auto bad = [&](const std::string& what) {
    throw std::runtime_error("manifest line " + std::to_string(n) + ": " + what);
  };
  while (std::getline(f, line)) {
    ++n;
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (n == 1) {
      if (line != "sifted-montecarlo 1") {
        bad("expected 'sifted-montecarlo 1'");
      }
      continue;
    }
    const std::vector<std::string> fl = split(line);
    if (n == 2 || n == 3) {
      if (fl.size() != 2 || fl[0] != (n == 2 ? "base_seed" : "generator")) {
        bad("expected base_seed and generator lines");
      }
      if (n == 2) {
        m.base_seed = number<std::uint64_t>(fl[1], "base seed");
      }
      continue;
    }
    if (n == 4) {
      if (line != "set,condition,scenario,run,seed,impairment_seed,p_detect,clutter_per_scan,dir") {
        bad("unexpected columns");
      }
      continue;
    }
    if (line.empty()) {
      continue;
    }
    if (fl.size() != 9) {
      bad("expected 9 fields");
    }
    ManifestRun r{fl[0],
                  fl[1],
                  fl[2],
                  number<int>(fl[3], "run"),
                  number<std::uint64_t>(fl[4], "seed"),
                  number<std::uint64_t>(fl[5], "seed"),
                  opt_number(fl[6], "p_detect"),
                  opt_number(fl[7], "clutter"),
                  fl[8]};
    if ((r.set != "noise" && r.set != "sweep") || r.scenario.empty() || r.run < 0 ||
        !safe_relative(r.dir) || !dirs.insert(r.dir).second ||
        r.p_detect.has_value() != r.clutter.has_value()) {
      bad("invalid row: " + line);
    }
    m.runs.push_back(std::move(r));
  }
  if (n < 4 || m.runs.empty()) {
    throw std::runtime_error("manifest " + path + " has no runs");
  }
  return m;
}

// ---- per-run metrics ------------------------------------------------------------

struct Metrics {
  std::optional<double> mean_rmse;  // over tracked targets; none if none tracked
  std::optional<double> max_rmse;
  std::size_t expected_targets = 0;  // with RMSE-eligible instants
  std::vector<int> untracked;        // expected but never matched past burn-in
  std::optional<double> missed_pct;  // none without eligible target-instants
  double id_switches = 0.0;
  double false_tracks = 0.0;
  double gospa = 0.0;      // RMS over the scored instants [m] (p = 2), generally (mean d^p)^(1/p)
  double gospa_loc = 0.0;  // mean per instant, m^p
  double gospa_missed = 0.0;    // mean per instant, m^p
  double gospa_false = 0.0;     // mean per instant, m^p
  double gospa_missed_n = 0.0;  // mean missed targets per instant
  double gospa_false_n = 0.0;   // mean false tracks per instant
  std::uint64_t track_limit = 0;
  bool engine_metrics = true;
};

Metrics metrics_of(const Score& s) {
  Metrics m;
  if (!s.rmse.empty()) {
    m.mean_rmse = s.mean_rmse;
    m.max_rmse = s.max_rmse;
  }
  for (const auto& [tgt, n] : s.rmse_eligible) {
    if (n == 0) {
      continue;
    }
    ++m.expected_targets;
    if (!s.rmse.contains(tgt)) {
      m.untracked.push_back(tgt);
    }
  }
  if (s.eligible > 0) {
    m.missed_pct = 100.0 * static_cast<double>(s.missed) / static_cast<double>(s.eligible);
  }
  m.id_switches = static_cast<double>(s.id_switches);
  m.false_tracks = static_cast<double>(s.false_tracks);
  const GospaSum& g = s.gospa;
  if (g.instants > 0) {
    const double n = static_cast<double>(g.instants);
    m.gospa = g.mean_total(s.gospa_params.p);
    m.gospa_loc = g.localisation / n;
    m.gospa_missed = g.missed / n;
    m.gospa_false = g.false_tracks / n;
    m.gospa_missed_n = static_cast<double>(g.missed_count) / n;
    m.gospa_false_n = static_cast<double>(g.false_count) / n;
  }
  m.track_limit = s.track_limit;
  m.engine_metrics = s.engine_metrics;
  return m;
}

// ---- trackers and conditions ----------------------------------------------------

struct Tracker {
  std::string name;
  std::optional<EngineConfig> engine;  // in process; else external
  std::string dir;                     // external: track report directory
  std::string info;                    // external: DIR/tracker.txt, if any
};

struct Condition {
  std::string set;
  std::string id;  // manifest condition, or "network" for the pipeline variant
  std::string label;
  bool network = false;
};

std::string fixed(double v, int precision) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", precision, v);
  std::string s = buf;
  if (s.find_first_not_of("-0.") == std::string::npos && s.front() == '-') {
    s.erase(0, 1);
  }
  return s;
}

std::string shortest(double v) {
  char buf[32];
  const auto r = std::to_chars(buf, buf + sizeof buf, v);
  return std::string(buf, r.ptr);
}

std::string condition_label(const ManifestRun& r) {
  if (r.set == "noise") {
    return "as defined";
  }
  return "clutter " + shortest(*r.clutter) + " / Pd " + shortest(*r.p_detect);
}

// Key of one evaluated run: tracker, condition, scenario, run index.
using RunKey = std::tuple<std::size_t, std::size_t, std::string, int>;

struct Results {
  std::vector<Tracker> trackers;
  std::vector<Condition> conditions;
  std::vector<std::string> scenarios;  // sorted
  std::map<RunKey, Metrics> runs;
  std::map<std::string, int> runs_per_scenario_noise;
  std::uint64_t base_seed = 0;
  std::size_t manifest_runs = 0;
};

std::string read_info(const std::string& dir) {
  const fs::path p = fs::path(dir) / "tracker.txt";
  std::error_code ec;
  if (!fs::is_regular_file(p, ec)) {
    return "";
  }
  if (fs::file_size(p, ec) > 4096 || ec) {
    throw std::runtime_error(p.string() + ": larger than 4096 bytes");
  }
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  std::string s = ss.str();
  for (const char c : s) {
    if (c != '\n' && (c < 0x20 || c > 0x7e)) {
      throw std::runtime_error(p.string() + ": not printable ASCII");
    }
  }
  while (!s.empty() && s.back() == '\n') {
    s.pop_back();
  }
  return s;
}

double engine_seconds(const Scenario& sc, const EngineConfig& cfg) {
  // What the report needs from a tracker: every measurement consumed in
  // order, scan by scan, and the tracks extrapolated to every truth instant.
  const auto start = std::chrono::steady_clock::now();
  Engine engine(sc.sensors, cfg);
  std::vector<MeasurementResult> results;
  std::vector<std::int64_t> times;
  for (const TruthRow& t : sc.truth) {
    if (times.empty() || times.back() != t.time_ns) {
      times.push_back(t.time_ns);
    }
  }
  std::size_t next = 0;
  std::size_t reported = 0;
  for (const std::int64_t t : times) {
    std::size_t end = next;
    while (end < sc.measurements.size() && sc.measurements[end].time.ns <= t) {
      ++end;
    }
    engine.on_scan(std::span<const Measurement>(sc.measurements).subspan(next, end - next),
                   results);
    next = end;
    reported += engine.tracks_at(Timestamp{t}).size();
  }
  const auto stop = std::chrono::steady_clock::now();
  if (reported == std::numeric_limits<std::size_t>::max()) {
    std::fputs("", stderr);  // keep the work observable to the optimiser
  }
  return std::chrono::duration<double>(stop - start).count();
}

Results evaluate(const Options& o, std::string& timing_csv) {
  const Manifest m = read_manifest(o.manifest);
  Results res;
  res.base_seed = m.base_seed;
  res.manifest_runs = m.runs.size();
  for (const std::string& t : o.trackers) {
    res.trackers.push_back({t, tracker_config(t), "", ""});
  }
  for (const External& e : o.externals) {
    res.trackers.push_back({e.name, std::nullopt, e.dir, read_info(e.dir)});
  }
  // Conditions in manifest order; the network variant right after the noise set.
  std::map<std::pair<std::string, std::string>, std::size_t> cond_index;
  const bool network = o.network != "none" && !o.trackers.empty();
  for (const ManifestRun& r : m.runs) {
    if (cond_index.emplace(std::pair{r.set, r.condition}, res.conditions.size()).second) {
      res.conditions.push_back({r.set, r.condition, condition_label(r), false});
      if (r.set == "noise" && network) {
        cond_index.emplace(std::pair{r.set, "network"}, res.conditions.size());
        res.conditions.push_back({r.set, "network",
                                  "as defined, " + o.network + " network (" + fixed(o.window_s, 2) +
                                      " s reorder window)",
                                  true});
      }
    }
  }
  std::set<std::string> scen;
  for (const ManifestRun& r : m.runs) {
    scen.insert(r.scenario);
    if (r.set == "noise" && r.condition == "default") {
      ++res.runs_per_scenario_noise[r.scenario];
    }
  }
  res.scenarios.assign(scen.begin(), scen.end());
  if (!o.export_dir.empty() && o.trackers.empty()) {
    throw std::invalid_argument("--export-tracks needs an in-process tracker");
  }
  timing_csv = "tracker,set,condition,scenario,run,seconds\n";

  for (const ManifestRun& r : m.runs) {
    const fs::path dir = m.root / r.dir;
    const Scenario sc = load_scenario_dir(dir.string());
    if (sc.truth.empty()) {
      throw std::runtime_error(dir.string() + ": truth.csv has no rows");
    }
    const std::size_t ci = cond_index.at({r.set, r.condition});
    for (std::size_t ti = 0; ti < res.trackers.size(); ++ti) {
      const Tracker& t = res.trackers[ti];
      if (!t.engine) {
        const std::string file = (fs::path(t.dir) / (r.dir + ".tracks")).string();
        const Score s =
            score_track_report(sc.truth, read_track_report_file(file), kMatchDistanceM, o.gospa);
        res.runs[{ti, ci, r.scenario, r.run}] = metrics_of(s);
        continue;
      }
      TrackReport exported;
      InstantObserver observe;
      if (ti == 0 && !o.export_dir.empty()) {
        observe = [&](std::int64_t at, const Engine& e) { append_engine_tracks(exported, at, e); };
      }
      const Score s = run_engine(sc, sc.measurements, *t.engine, kMatchDistanceM, o.gospa, observe);
      res.runs[{ti, ci, r.scenario, r.run}] = metrics_of(s);
      if (observe) {
        const fs::path out = fs::path(o.export_dir) / (r.dir + ".tracks");
        fs::create_directories(out.parent_path());
        std::ofstream f(out, std::ios::binary | std::ios::trunc);
        f << format_track_report(exported);
        if (!f.good()) {
          throw std::runtime_error("cannot write " + out.string());
        }
      }
      if (!o.timing.empty()) {
        timing_csv += t.name + "," + r.set + "," + r.condition + "," + r.scenario + "," +
                      std::to_string(r.run) + "," + fixed(engine_seconds(sc, *t.engine), 6) + "\n";
      }
      if (r.set == "noise" && network) {
        net::ImpairmentConfig ic;
        ic.seed = r.impairment_seed;
        ic.global = impairment_level(o.network);
        net::PipelineConfig pc;
        pc.engine = *t.engine;
        pc.reorder_window_s = o.window_s;
        const PipelineRun p = run_pipeline(sc, net::impair(sent_frames(sc.measurements), ic), pc,
                                           kDaemonTickNs, kMatchDistanceM, o.gospa);
        res.runs[{ti, cond_index.at({r.set, "network"}), r.scenario, r.run}] = metrics_of(p.score);
      }
    }
  }
  return res;
}

// ---- statistics -----------------------------------------------------------------

struct Summary {
  std::size_t runs = 0;
  std::optional<Estimate> rmse;
  std::size_t rmse_runs = 0;
  std::optional<double> worst_median;
  std::optional<double> worst_max;
  std::size_t expected = 0;   // target-runs expected to be tracked
  std::size_t untracked = 0;  // ... never tracked
  std::optional<Estimate> missed;
  Estimate id_switches;
  Estimate id_any;
  Estimate false_tracks;
  Estimate false_any;
  Estimate gospa;
  double gospa_loc = 0.0;
  double gospa_missed = 0.0;
  double gospa_false = 0.0;
  double gospa_missed_n = 0.0;
  double gospa_false_n = 0.0;
  std::uint64_t track_limit = 0;
  std::size_t track_limit_runs = 0;
  bool engine_metrics = true;
};

Summary summarize(const std::vector<const Metrics*>& ms, const Options& o,
                  const std::string& label) {
  auto seed = [&](const char* metric) { return seed_for(o.stats_seed, label + "/" + metric); };
  Summary s;
  s.runs = ms.size();
  std::vector<double> rmse;
  std::vector<double> worst;
  std::vector<double> missed;
  std::vector<double> idsw;
  std::vector<double> fals;
  std::vector<double> gospa;
  std::size_t id_any = 0;
  std::size_t false_any = 0;
  for (const Metrics* m : ms) {
    if (m->mean_rmse) {
      rmse.push_back(*m->mean_rmse);
      worst.push_back(*m->max_rmse);
    }
    if (m->missed_pct) {
      missed.push_back(*m->missed_pct);
    }
    s.expected += m->expected_targets;
    s.untracked += m->untracked.size();
    idsw.push_back(m->id_switches);
    fals.push_back(m->false_tracks);
    gospa.push_back(m->gospa);
    id_any += m->id_switches > 0.0 ? 1U : 0U;
    false_any += m->false_tracks > 0.0 ? 1U : 0U;
    s.gospa_loc += m->gospa_loc;
    s.gospa_missed += m->gospa_missed;
    s.gospa_false += m->gospa_false;
    s.gospa_missed_n += m->gospa_missed_n;
    s.gospa_false_n += m->gospa_false_n;
    s.track_limit += m->track_limit;
    s.track_limit_runs += m->track_limit > 0 ? 1U : 0U;
    s.engine_metrics = s.engine_metrics && m->engine_metrics;
  }
  const double n = static_cast<double>(ms.size());
  s.gospa_loc /= n;
  s.gospa_missed /= n;
  s.gospa_false /= n;
  s.gospa_missed_n /= n;
  s.gospa_false_n /= n;
  s.rmse_runs = rmse.size();
  if (!rmse.empty()) {
    s.rmse = bootstrap_mean(rmse, seed("rmse"), o.bootstrap);
    s.worst_median = quantile(worst, 0.5);
    s.worst_max = quantile(worst, 1.0);
  }
  if (!missed.empty()) {
    s.missed = bootstrap_mean(missed, seed("missed"), o.bootstrap);
  }
  s.id_switches = count_mean(idsw, seed("id_switches"), o.bootstrap);
  s.false_tracks = count_mean(fals, seed("false_tracks"), o.bootstrap);
  s.gospa = bootstrap_mean(gospa, seed("gospa"), o.bootstrap);
  s.id_any = wilson(id_any, ms.size());
  s.false_any = wilson(false_any, ms.size());
  return s;
}

// Selection of runs: tracker, condition, scenario ("" = every scenario),
// runs with index < max_run.
std::vector<const Metrics*> select(const Results& r, std::size_t ti, std::size_t ci,
                                   const std::string& scenario,
                                   int max_run = std::numeric_limits<int>::max()) {
  std::vector<const Metrics*> out;
  for (const auto& [key, m] : r.runs) {
    const auto& [t, c, sc, run] = key;
    if (t == ti && c == ci && (scenario.empty() || sc == scenario) && run < max_run) {
      out.push_back(&m);
    }
  }
  return out;
}

// Per-run values of one metric for two (tracker, condition) selections,
// paired by (scenario, run); runs missing from either side are left out.
using Getter = std::optional<double> (*)(const Metrics&);
struct Paired {
  std::vector<double> a;
  std::vector<double> b;
};
Paired pair_up(const Results& r, std::size_t ta, std::size_t ca, std::size_t tb, std::size_t cb,
               const std::string& scenario, Getter get) {
  Paired p;
  for (const auto& [key, m] : r.runs) {
    const auto& [t, c, sc, run] = key;
    if (t != ta || c != ca || (!scenario.empty() && sc != scenario)) {
      continue;
    }
    const auto other = r.runs.find({tb, cb, sc, run});
    if (other == r.runs.end()) {
      continue;
    }
    const auto x = get(m);
    const auto y = get(other->second);
    if (x && y) {
      p.a.push_back(*x);
      p.b.push_back(*y);
    }
  }
  return p;
}

struct MetricDef {
  const char* name;
  const char* unit;
  int precision;
  Getter get;
};
const MetricDef kPairedMetrics[] = {
    {"mean RMSE", "m", 2, [](const Metrics& m) { return m.mean_rmse; }},
    {"missed", "%", 2, [](const Metrics& m) { return m.missed_pct; }},
    {"ID switches", "/run", 3,
     [](const Metrics& m) { return std::optional<double>(m.id_switches); }},
    {"false tracks", "/run", 2,
     [](const Metrics& m) { return std::optional<double>(m.false_tracks); }},
    {"GOSPA", "m", 2, [](const Metrics& m) { return std::optional<double>(m.gospa); }},
};

// ---- formatting -----------------------------------------------------------------

const std::string kDash = "–";

std::string interval(const Estimate& e, int p) {
  if (e.exact_zero_bound) {
    return "0 [0, " + fixed(e.hi, p) + "]";
  }
  return fixed(e.value, p) + " [" + fixed(e.lo, p) + ", " + fixed(e.hi, p) + "]";
}

std::string interval(const std::optional<Estimate>& e, int p) {
  return e ? interval(*e, p) : kDash;
}

std::string percent(const Estimate& e) {
  return fixed(100.0 * e.value, 1) + " [" + fixed(100.0 * e.lo, 1) + ", " + fixed(100.0 * e.hi, 1) +
         "]";
}

std::string verdict(const Estimate& d, bool lower_is_better) {
  if (d.lo <= 0.0 && d.hi >= 0.0) {
    return "no detectable difference";
  }
  const bool lower = d.hi < 0.0;
  return lower == lower_is_better ? "**better**" : "**worse**";
}

std::string summary_header(bool with_tracker, bool with_scenario) {
  std::string h = "|";
  std::string rule = "|";
  if (with_tracker) {
    h += " tracker |";
    rule += "---|";
  }
  if (with_scenario) {
    h += " scenario |";
    rule += "---|";
  }
  h += " condition | runs | mean RMSE m | worst target RMSE m (median / max) | never tracked | "
       "missed % | ID switches / run | runs with an ID switch % | false tracks / run | runs "
       "with a false track % | GOSPA m | TrackLimit (runs) |\n";
  rule += "---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  return h + rule;
}

std::string summary_row(const Summary& s, const std::string& prefix, const std::string& cond) {
  std::string worst =
      s.worst_median ? fixed(*s.worst_median, 2) + " / " + fixed(*s.worst_max, 2) : kDash;
  return "| " + prefix + cond + " | " + std::to_string(s.runs) + " | " + interval(s.rmse, 2) +
         " | " + worst + " | " + std::to_string(s.untracked) + "/" + std::to_string(s.expected) +
         " | " + interval(s.missed, 2) + " | " + interval(s.id_switches, 3) + " | " +
         percent(s.id_any) + " | " + interval(s.false_tracks, 2) + " | " + percent(s.false_any) +
         " | " + interval(s.gospa, 2) + " | " +
         (s.engine_metrics
              ? std::to_string(s.track_limit) + " (" + std::to_string(s.track_limit_runs) + ")"
              : "n/a") +
         " |\n";
}

std::string gospa_row(const Summary& s, const std::string& prefix, const std::string& cond,
                      double p) {
  const double total = s.gospa_loc + s.gospa_missed + s.gospa_false;
  auto share = [&](double v) { return total > 0.0 ? fixed(100.0 * v / total, 1) : kDash; };
  return "| " + prefix + cond + " | " + interval(s.gospa, 2) + " | " +
         fixed(std::pow(total, 1.0 / p), 2) + " | " + share(s.gospa_loc) + " | " +
         share(s.gospa_missed) + " | " + share(s.gospa_false) + " | " + fixed(s.gospa_missed_n, 3) +
         " | " + fixed(s.gospa_false_n, 3) + " |\n";
}

std::string command_line(const Options& o) {
  std::string s = "sifted_montecarlo";
  for (const std::string& a : o.args) {
    s += " " + a;
  }
  return s;
}

std::size_t tracker_index(const Results& r, const std::string& name) {
  for (std::size_t i = 0; i < r.trackers.size(); ++i) {
    if (r.trackers[i].name == name) {
      return i;
    }
  }
  throw std::logic_error("no tracker " + name);
}

std::optional<std::size_t> condition_index(const Results& r, const std::string& set,
                                           const std::string& id) {
  for (std::size_t i = 0; i < r.conditions.size(); ++i) {
    if (r.conditions[i].set == set && r.conditions[i].id == id) {
      return i;
    }
  }
  return std::nullopt;
}

std::string paired_table(const Results& r, const Options& o, std::size_t ta, std::size_t ca,
                         std::size_t tb, std::size_t cb, const std::string& scenario,
                         const std::string& label) {
  std::string out;
  for (const MetricDef& md : kPairedMetrics) {
    const Paired p = pair_up(r, ta, ca, tb, cb, scenario, md.get);
    if (p.a.empty()) {
      continue;
    }
    const Estimate d =
        paired_difference(p.a, p.b, seed_for(o.stats_seed, label + md.name), o.bootstrap);
    out += "| " + label + " | " + md.name + " " + md.unit + " | " + std::to_string(p.a.size()) +
           " | " + interval(d, md.precision) + " | " + verdict(d, true) + " |\n";
  }
  return out;
}

// The paired difference paired_table prints for one metric (same seed, so the
// same numbers), or none without paired runs.
std::optional<Estimate> paired_estimate(const Results& r, const Options& o, std::size_t ta,
                                        std::size_t ca, std::size_t tb, std::size_t cb,
                                        const std::string& label, const MetricDef& md) {
  const Paired p = pair_up(r, ta, ca, tb, cb, "", md.get);
  if (p.a.empty()) {
    return std::nullopt;
  }
  return paired_difference(p.a, p.b, seed_for(o.stats_seed, label + md.name), o.bootstrap);
}

const char* kPairedHeader =
    "| comparison | metric | paired runs | difference (first − second) [95% CI] | first is |\n"
    "|---|---|---:|---:|---|\n";

std::string convergence_table(const Results& r, const Options& o, std::size_t ti, std::size_t ci,
                              const std::vector<std::string>& scenarios, int max_runs) {
  std::string out =
      "| selection | runs | ID switches / run | width | GOSPA m | width | mean RMSE m | width |\n"
      "|---|---:|---:|---:|---:|---:|---:|---:|\n";
  std::vector<int> ks;
  for (const int k : {10, 20, 50, 100, 200, 500}) {
    if (k < max_runs) {
      ks.push_back(k);
    }
  }
  ks.push_back(max_runs);
  for (const std::string& sc : scenarios) {
    for (const int k : ks) {
      const auto ms = select(r, ti, ci, sc, k);
      if (ms.empty()) {
        continue;
      }
      const Summary s = summarize(ms, o, "convergence/" + sc + "/" + std::to_string(k));
      auto w = [](const std::optional<Estimate>& e) { return e ? fixed(e->hi - e->lo, 3) : kDash; };
      out += "| " + (sc.empty() ? std::string("all scenarios") : sc) + ", first " +
             std::to_string(k) + " per scenario | " + std::to_string(s.runs) + " | " +
             interval(s.id_switches, 3) + " | " + fixed(s.id_switches.hi - s.id_switches.lo, 3) +
             " | " + interval(s.gospa, 2) + " | " + fixed(s.gospa.hi - s.gospa.lo, 3) + " | " +
             interval(s.rmse, 2) + " | " + w(s.rmse) + " |\n";
    }
  }
  return out;
}

std::string untracked_list(const Results& r) {
  std::vector<std::string> rows;
  std::size_t total = 0;
  for (const auto& [key, m] : r.runs) {
    const auto& [t, c, sc, run] = key;
    for (const int tgt : m.untracked) {
      ++total;
      if (rows.size() < 25) {
        rows.push_back("| " + r.trackers[t].name + " | " + r.conditions[c].label + " | " + sc +
                       " | " + std::to_string(run) + " | " + std::to_string(tgt) + " |\n");
      }
    }
  }
  if (total == 0) {
    return "No target was left untracked in any run: every target that should have been "
           "tracked has an RMSE in every run.\n";
  }
  std::string out = std::to_string(total) +
                    " target-runs had no matched instant past the RMSE burn-in, so their RMSE "
                    "is undefined. They are left out of the RMSE columns and counted in the "
                    "\"never tracked\" column instead.";
  out += total > rows.size() ? " The first " + std::to_string(rows.size()) + ":\n\n" : "\n\n";
  out += "| tracker | condition | scenario | run | target |\n|---|---|---|---:|---:|\n";
  for (const std::string& row : rows) {
    out += row;
  }
  return out;
}

std::string definitions(const Options& o) {
  return "\n## Method and metric definitions\n\n"
         "- **Unit of analysis: the run.** Every interval is computed from one value per run "
         "(a run's RMSE, ID-switch count, GOSPA, ...), never from time samples within a run, "
         "which are strongly correlated.\n"
         "- **Point estimates and intervals.** Mean over runs with a 95% seeded percentile "
         "bootstrap interval (" +
         std::to_string(o.bootstrap) + " resamples, base seed " + std::to_string(o.stats_seed) +
         "; `eval/sifted/eval/stats.hpp`). Counts (ID switches, false tracks) are skewed, and "
         "the bootstrap makes no normality assumption. With no event in n runs it shows "
         "`0 [0, u]`: u = -ln(0.025)/n = 3.69/n is the exact Poisson 95% upper bound on the "
         "mean per run. Fractions of runs (\"runs with an ID switch\") use the Wilson score "
         "interval.\n"
         "- **Paired comparisons** use common random numbers: run r of a scenario has the same "
         "sensor noise, detections and clutter in every condition and for every tracker, so "
         "the difference is taken run by run and the interval is the bootstrap of the mean "
         "difference. \"No detectable difference\" means the interval contains 0.\n"
         "- **Scoring** is the scorecard's own (`eval/sifted/eval/scenario.hpp`, "
         "[evaluation.md](evaluation.md)): CLEAR-MOT style matching within " +
         fixed(kMatchDistanceM, 0) +
         " m. RMSE per target over covered, matched instants at "
         "least " +
         fixed(kRmseBurnInS, 0) +
         " s after the target enters coverage; a run's mean RMSE averages its tracked targets "
         "and its worst RMSE is its largest. Missed %: covered target-instants at least " +
         fixed(kMissBurnInS, 0) +
         " s after entering coverage with no matched track. ID switch: a target matched to a "
         "different track than the last time it was matched. False track: a reported track "
         "matched to a target in fewer than half of the instants it was reported.\n"
         "- **GOSPA** (alpha = 2; Rahmathullah, García-Fernández and Svensson, FUSION 2017, "
         "arXiv:1601.05585; `eval/sifted/eval/gospa.hpp`) with **c = " +
         shortest(o.gospa.c) + " m, p = " + shortest(o.gospa.p) + "**, per truth instant from " +
         fixed(kMissBurnInS, 0) +
         " s after the first, between the targets counted for \"missed\" and the reported "
         "(confirmed and coasting) tracks. A track matched to a target that is present but out "
         "of coverage or still in its burn-in is left out of both sets. c equals the matching "
         "distance, so a GOSPA miss or false track is what the CLEAR-MOT matcher would call "
         "one (sized at about 5x the filtered radar error at the 8 km edge of coverage and "
         "below the closest lane spacing, 120 m). p = 2 weighs localisation errors as RMSE "
         "does. A run's GOSPA is (mean over instants of d^p)^(1/p), the RMS GOSPA for p = 2. "
         "The decomposition splits the mean d^p into localisation (assigned pairs), missed "
         "targets and false tracks (c^p/2 each), exactly additive.\n"
         "- **Never tracked**: target-runs where the target was covered past the RMSE burn-in "
         "but never matched. Their RMSE is undefined. They are counted here, not dropped "
         "silently, and the RMSE columns average only the runs and targets that have one.\n"
         "- **TrackLimit**: measurements that could not start a track because the engine was at "
         "`max_tracks` (1024), summed over runs (runs with any). Not available (n/a) for "
         "external trackers.\n"
         "- **Determinism.** The same binary on the same architecture, given the same manifest "
         "(scenario code and base seed), writes the same bytes. Different compilers or maths "
         "libraries may change the last bits of the simulated noise (Python `sin`/`cos`) or of "
         "the filters, and so individual runs. The intervals are meant to absorb exactly that "
         "kind of variation.\n";
}

// Plain-language findings, every number computed from the results above (the
// mechanisms are the design's, stated as such; nothing names a scenario).
std::string findings(const Results& r, const Options& o,
                     const std::map<std::pair<std::size_t, std::size_t>, Summary>& pooled,
                     std::size_t ref, std::optional<std::size_t> noise,
                     std::optional<std::size_t> network) {
  std::string out = "\n## Findings\n\n";
  const std::string& name = r.trackers[ref].name;
  if (o.baseline_page) {
    for (std::size_t ti = 0; ti < r.trackers.size(); ++ti) {
      if (ti == ref) {
        continue;
      }
      std::vector<std::string> better;
      std::vector<std::string> worse;
      std::size_t same = 0;
      for (std::size_t ci = 0; ci < r.conditions.size(); ++ci) {
        const std::string label =
            r.trackers[ti].name + " − " + name + ", " + r.conditions[ci].label;
        for (const MetricDef& md : kPairedMetrics) {
          const auto d = paired_estimate(r, o, ti, ci, ref, ci, label, md);
          if (!d) {
            continue;
          }
          const std::string item = std::string(md.name) + " at " + r.conditions[ci].label + " (" +
                                   interval(*d, md.precision) + " " + md.unit + ")";
          if (d->hi < 0.0) {
            better.push_back(item);
          } else if (d->lo > 0.0) {
            worse.push_back(item);
          } else {
            ++same;
          }
        }
      }
      out += "- **" + r.trackers[ti].name + " against " + name +
             "**: " + std::to_string(better.size()) + " condition-metric pairs better, " +
             std::to_string(worse.size()) + " worse, " + std::to_string(same) +
             " with no detectable difference.";
      auto list = [&](const std::string& what, const std::vector<std::string>& items) {
        if (items.empty()) {
          return;
        }
        out += " " + what + ": ";
        for (std::size_t i = 0; i < items.size(); ++i) {
          out += (i ? "; " : "") + items[i];
        }
        out += ".";
      };
      list(r.trackers[ti].name + " is better on", better);
      list(name + " is better on", worse);
      out += "\n";
    }
    out +=
        "- Every difference above is paired on common random numbers (same run, same noise). "
        "A condition with fewer runs has a wider interval, and \"no detectable difference\" "
        "there is weaker evidence of equality.\n";
    return out;
  }

  if (noise) {
    const Summary& s = pooled.at({ref, *noise});
    out += "- **Accuracy over fresh noise.** Over " + std::to_string(s.runs) +
           " independent realisations, " + name + "'s mean RMSE is " + interval(s.rmse, 2) +
           " m. The worst target RMSE of a run has median " + fixed(*s.worst_median, 2) +
           " m, but the worst single run reaches " + fixed(*s.worst_max, 2) +
           " m: one noise draw per scenario cannot show that tail.\n";
    std::vector<std::pair<double, std::string>> by;
    for (const std::string& sc : r.scenarios) {
      const auto ms = select(r, ref, *noise, sc);
      if (ms.empty()) {
        continue;
      }
      const Summary ss = summarize(ms, o, name + "/noise/" + r.conditions[*noise].id + "/" + sc);
      by.emplace_back(ss.id_any.value, "`" + sc + "` " + percent(ss.id_any) + "% of runs (" +
                                           interval(ss.id_switches, 2) + " per run)");
    }
    std::stable_sort(by.begin(), by.end(),
                     [](const auto& a, const auto& b) { return a.first > b.first; });
    out += "- **Identity breaks.** " + percent(s.id_any) +
           "% of runs break identity at least once (" + interval(s.id_switches, 3) +
           " ID switches per run). By scenario: ";
    for (std::size_t i = 0; i < by.size(); ++i) {
      out += (i ? "; " : "") + by[i].second;
    }
    out +=
        ". Breaks concentrate where targets fly closer than a few measurement sigmas for "
        "many scans in a row. Their gates then overlap, and on a scan whose noise makes the "
        "swapped pairing cheaper, even the global (Hungarian) assignment swaps the tracks. "
        "Once swapped, each track follows the other target, and the scorer counts the switch "
        "and the larger error. The average hides how uneven this is: most runs have none, "
        "a few have several.\n";
  }
  if (noise && network) {
    const std::string label = "network − direct, " + name;
    const auto d_rmse = paired_estimate(r, o, ref, *network, ref, *noise, label, kPairedMetrics[0]);
    const auto d_id = paired_estimate(r, o, ref, *network, ref, *noise, label, kPairedMetrics[2]);
    if (d_rmse && d_id) {
      out += "- **Network impairment.** Through the pipeline under " + o.network +
             " impairment, the same runs score " + interval(*d_rmse, 2) +
             " m higher mean RMSE and " + interval(*d_id, 3) +
             " more ID switches per run than the direct feed (paired).\n";
    }
  }
  // Clutter: false tracks per run by clutter level, for each detection probability.
  std::map<std::string, std::vector<std::pair<double, const Summary*>>> by_pd;
  for (std::size_t ci = 0; ci < r.conditions.size(); ++ci) {
    const Condition& c = r.conditions[ci];
    const auto it = pooled.find({ref, ci});
    if (c.set != "sweep" || it == pooled.end()) {
      continue;
    }
    const std::size_t slash = c.label.find(" / ");
    const double clutter = number<double>(c.label.substr(8, slash - 8), "clutter");
    by_pd[c.label.substr(slash + 3)].emplace_back(clutter, &it->second);
  }
  for (auto& [pd, rows] : by_pd) {
    std::sort(rows.begin(), rows.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    out += "- **Clutter, " + pd + ".** False tracks per run: ";
    for (std::size_t i = 0; i < rows.size(); ++i) {
      out += (i ? ", " : "") + interval(rows[i].second->false_tracks, 2) + " at " +
             shortest(rows[i].first);
    }
    out += " clutter points per scan.";
    // Growth from the lowest level with at least kGrowthBase false tracks per
    // run: a ratio against a near-zero mean says nothing.
    constexpr double kGrowthBase = 0.5;
    const auto base = std::find_if(rows.begin(), rows.end(), [&](const auto& x) {
      return x.first > 0.0 && x.second->false_tracks.value >= kGrowthBase;
    });
    if (base != rows.end() && base + 1 < rows.end()) {
      const double f = rows.back().second->false_tracks.value / base->second->false_tracks.value;
      const double c = rows.back().first / base->first;
      out += std::string(" From ") + shortest(base->first) + " to " + shortest(rows.back().first) +
             " points per scan (x" + fixed(c, 0) + " the clutter) they grow x" + fixed(f, 0) +
             (f > c ? ", faster than the clutter itself" : "") + ".";
    }
    std::uint64_t limit = 0;
    std::optional<double> lo_rmse;
    std::optional<double> hi_rmse;
    for (const auto& [cl, s] : rows) {
      limit += s->track_limit;
      if (s->rmse) {
        lo_rmse = std::min(lo_rmse.value_or(s->rmse->value), s->rmse->value);
        hi_rmse = std::max(hi_rmse.value_or(s->rmse->value), s->rmse->value);
      }
    }
    if (lo_rmse) {
      out += " Mean RMSE on the true targets stays between " + fixed(*lo_rmse, 2) + " and " +
             fixed(*hi_rmse, 2) + " m.";
    }
    out += limit == 0 ? std::string(" The track cap was never reached (TrackLimit 0).")
                      : " " + std::to_string(limit) + " measurements hit the track cap.";
    out += "\n";
  }
  if (!by_pd.empty()) {
    out +=
        "- **Why false tracks grow with clutter.** Any range-bearing detection that no "
        "established track takes starts a tentative track, and a tentative track is confirmed "
        "after 3 hits in its chi-square gate with no gap over 3 s. Clutter is uniform over the "
        "coverage disc, so both the number of clutter-born tentative tracks and the chance "
        "that clutter falls in their gates on the next scans rise with clutter density, and "
        "the confirmations multiply. GNN still gives the true detection to the true target's "
        "track, which is why accuracy on real targets barely moves. The GOSPA decomposition "
        "shows the cost: under heavy clutter almost all of GOSPA is false tracks. This is the "
        "confirmation logic's limit (3-of-3 at 1 Hz), not a failure of association. The "
        "baseline comparison isolates the motion model's part in it.\n";
  }
  return out;
}

std::string markdown(const Options& o, const Results& r) {
  std::string out;
  const std::size_t ref = tracker_index(r, o.reference);
  const auto noise = condition_index(r, "noise", "default");
  const auto network = condition_index(r, "noise", "network");
  if (o.baseline_page) {
    out += "# Baseline comparison: Sifted and Stone Soup\n\n";
    out +=
        "<!-- Generated by sifted_montecarlo (eval/sifted_montecarlo.cpp) via "
        "benchmarks/baselines/stonesoup/compare.py. Do not edit by hand. -->\n\n";
    out +=
        "Sifted against an independent tracker built from an established open-source framework "
        "(Stone Soup), on the same Monte Carlo runs and the same input files, scored by the "
        "same scorer (Stone Soup's tracks go through the track report interchange format, "
        "`eval/sifted/eval/track_report.hpp`). Pairing is by run (common random numbers). How "
        "the trackers are configured, and every place where they cannot be matched exactly, is "
        "in [benchmarks/baselines/stonesoup/README.md](../benchmarks/baselines/stonesoup/"
        "README.md) and ADR 0015. No tracker was tuned on these runs. The Sifted-only Monte "
        "Carlo evaluation, with more runs and the network condition, is "
        "[evaluation-monte-carlo.md](evaluation-monte-carlo.md). Processing time is machine "
        "dependent and so is not on this page: see "
        "[benchmarks/baseline-timing.md](benchmarks/baseline-timing.md).\n\n";
  } else {
    out += "# Monte Carlo evaluation\n\n";
    out +=
        "<!-- Generated by sifted_montecarlo (eval/sifted_montecarlo.cpp) via "
        "`python3 -m sifted_sim.montecarlo report`. Do not edit by hand. -->\n\n";
    out +=
        "The scorecard ([evaluation.md](evaluation.md)) scores one sensor-noise realisation per "
        "scenario. Its seeds vary only the network, so a zero there may be the luck of one "
        "draw. This page scores many independent realisations of every multi-target scenario "
        "and a clutter and detection-probability sweep. Each metric is reported as a mean over "
        "runs with a 95% interval, alongside how many runs had any identity break at all. The "
        "comparison with an external baseline tracker is "
        "[evaluation-baseline.md](evaluation-baseline.md).\n\n";
  }

  // What was run.
  out += "## What was run\n\n";
  out += "```bash\n";
  if (o.baseline_page) {
    out +=
        "build/stonesoup-venv/bin/python benchmarks/baselines/stonesoup/"
        "compare.py\n";
  } else {
    out += "cmake --preset release && cmake --build --preset release\n";
    out += "PYTHONPATH=sim python3 -m sifted_sim.montecarlo report\n";
  }
  out += "```\n\n";
  out += "which runs `" + command_line(o) + "` on the generated manifest. ";
  out += "Base seed " + std::to_string(r.base_seed) + ", " + std::to_string(r.manifest_runs) +
         " generated runs. Each run's sensor seeds derive from (base seed, scenario, run index) "
         "through SplitMix64 (`sim/sifted_sim/montecarlo.py`), the same in every condition. "
         "Scenarios: ";
  for (std::size_t i = 0; i < r.scenarios.size(); ++i) {
    out += (i ? ", " : "") + std::string("`") + r.scenarios[i] + "`";
  }
  out += " (definitions in `sim/sifted_sim/scenarios.py`).\n\n";
  out += "| tracker | what it is |\n|---|---|\n";
  for (const Tracker& t : r.trackers) {
    std::string what;
    if (t.name == "sifted") {
      what =
          "Sifted, `EngineConfig` defaults: IMM estimator (ADR 0014), GNN, chi-square gate "
          "0.999, 3-hit confirmation";
    } else if (t.name == "sifted-cv") {
      what =
          "Sifted with the legacy constant-velocity filter (q = 4 m^2/s^3) and its "
          "manoeuvre process-noise switching (x10 on a fading-memory NIS test)";
    } else if (t.name == "sifted-cv-fixed-q") {
      what =
          "Sifted, constant-velocity filter with fixed q = 4 m^2/s^3 (manoeuvre switching "
          "off): the like-for-like configuration for an EKF/CV baseline";
    } else {
      what = t.info.empty() ? "external tracker (track reports)" : t.info;
      std::replace(what.begin(), what.end(), '\n', ' ');
    }
    out += "| " + t.name + " | " + what + " |\n";
  }
  out += "\n| condition | set | runs per scenario | what changes |\n|---|---|---:|---|\n";
  for (std::size_t ci = 0; ci < r.conditions.size(); ++ci) {
    const Condition& c = r.conditions[ci];
    std::map<std::string, int> per;
    for (const auto& [key, m] : r.runs) {
      if (std::get<0>(key) == 0 && std::get<1>(key) == ci) {
        ++per[std::get<2>(key)];
      }
    }
    std::set<int> counts;
    for (const auto& [sc, n] : per) {
      counts.insert(n);
    }
    std::string n;
    for (const int k : counts) {
      n += (n.empty() ? "" : "/") + std::to_string(k);
    }
    std::string what = c.set == "noise"
                           ? (c.network ? "frames through the seeded impairment model and "
                                          "net::Pipeline as the daemon consumes them "
                                          "(`run_pipeline`); in-process trackers only"
                                        : "fresh noise, detections and clutter; scenario "
                                          "parameters unchanged; direct engine feed")
                           : "every radar's clutter (points per scan) and detection probability "
                             "replaced; cameras unchanged; direct feed";
    out += "| " + c.label + " | " + c.set + " | " + n + " | " + what + " |\n";
  }

  // Summary (computed first: the findings quote it).
  std::string summary;
  summary += "\n## Summary per condition (all scenarios pooled)\n\n";
  summary +=
      "Each cell is the mean over runs [95% interval]. \"Never tracked\" counts target-runs "
      "with no RMSE (see the method section).\n\n";
  summary += summary_header(true, false);
  std::map<std::pair<std::size_t, std::size_t>, Summary> pooled;
  for (std::size_t ci = 0; ci < r.conditions.size(); ++ci) {
    for (std::size_t ti = 0; ti < r.trackers.size(); ++ti) {
      const auto ms = select(r, ti, ci, "");
      if (ms.empty()) {
        continue;
      }
      const Summary s = summarize(
          ms, o,
          r.trackers[ti].name + "/" + r.conditions[ci].set + "/" + r.conditions[ci].id + "/all");
      pooled[{ti, ci}] = s;
      summary += summary_row(s, r.trackers[ti].name + " | ", r.conditions[ci].label);
    }
  }
  out += findings(r, o, pooled, ref, noise, network);
  out += summary;

  // Per scenario, noise set.
  if (noise) {
    out += "\n## Per scenario, fresh noise (as defined)\n\n";
    out += summary_header(true, true);
    for (const std::string& sc : r.scenarios) {
      for (std::size_t ti = 0; ti < r.trackers.size(); ++ti) {
        for (const auto ci : {noise, network}) {
          if (!ci) {
            continue;
          }
          const auto ms = select(r, ti, *ci, sc);
          if (ms.empty()) {
            continue;
          }
          const Summary s =
              summarize(ms, o, r.trackers[ti].name + "/noise/" + r.conditions[*ci].id + "/" + sc);
          out += summary_row(s, r.trackers[ti].name + " | " + sc + " | ", r.conditions[*ci].label);
        }
      }
    }
  }

  // GOSPA decomposition.
  out += "\n## GOSPA decomposition (all scenarios pooled)\n\n";
  out += "GOSPA (c = " + shortest(o.gospa.c) + " m, p = " + shortest(o.gospa.p) +
         ", alpha = 2): the run mean [95% interval], the value of the pooled mean d^p, the share "
         "of that mean d^p due to localisation, missed targets and false tracks, and the mean "
         "number of missed targets and false tracks per scored instant.\n\n";
  out +=
      "| tracker | condition | GOSPA m | pooled m | localisation % | missed % | false % | "
      "missed / instant | false / instant |\n|---|---|---:|---:|---:|---:|---:|---:|---:|\n";
  for (const auto& [key, s] : pooled) {
    out +=
        gospa_row(s, r.trackers[key.first].name + " | ", r.conditions[key.second].label, o.gospa.p);
  }

  // Paired comparisons.
  out += "\n## Paired comparisons\n\n";
  out +=
      "Differences of per-run values on common random numbers. A negative difference means "
      "the first is lower (better for every metric here).\n\n";
  out += kPairedHeader;
  if (o.baseline_page) {
    for (std::size_t ci = 0; ci < r.conditions.size(); ++ci) {
      for (std::size_t ti = 0; ti < r.trackers.size(); ++ti) {
        if (ti == ref) {
          continue;
        }
        out += paired_table(
            r, o, ti, ci, ref, ci, "",
            r.trackers[ti].name + " − " + r.trackers[ref].name + ", " + r.conditions[ci].label);
      }
    }
  } else {
    if (noise && network) {
      out += paired_table(r, o, ref, *network, ref, *noise, "",
                          "network − direct, " + r.trackers[ref].name);
    }
    for (std::size_t ci = 0; ci < r.conditions.size(); ++ci) {
      const Condition& c = r.conditions[ci];
      const std::string tag = "_pd0.7";
      if (c.set != "sweep" || c.id.size() < tag.size() ||
          c.id.compare(c.id.size() - tag.size(), tag.size(), tag) != 0) {
        continue;
      }
      const std::string base = c.id.substr(0, c.id.size() - tag.size()) + "_pd0.9";
      if (const auto cb = condition_index(r, "sweep", base)) {
        out += paired_table(r, o, ref, ci, ref, *cb, "",
                            "Pd 0.7 − Pd 0.9 at " + c.label.substr(0, c.label.find(" /")) + ", " +
                                r.trackers[ref].name);
      }
    }
  }

  // Convergence.
  if (noise) {
    int max_runs = 0;
    for (const auto& [sc, n] : r.runs_per_scenario_noise) {
      max_runs = std::max(max_runs, n);
    }
    out += "\n## Convergence: interval width against run count\n\n";
    out += "The noise set's first k runs per scenario, " + r.trackers[ref].name +
           ". For a mean the interval narrows roughly as 1/sqrt(runs), so quadrupling the runs "
           "halves it. This is the evidence for the number of runs chosen: by the last row the "
           "width is small against the differences the page reports.\n\n";
    std::vector<std::string> sel{""};
    sel.insert(sel.end(), r.scenarios.begin(), r.scenarios.end());
    out += convergence_table(r, o, ref, *noise, sel, max_runs);
  }

  out += "\n## Targets never tracked\n\n" + untracked_list(r);
  out += definitions(o);
  return out;
}

std::string csv(const Results& r) {
  std::string out =
      "tracker,set,condition,scenario,run,mean_rmse_m,max_rmse_m,expected_targets,"
      "untracked_targets,missed_pct,id_switches,false_tracks,gospa_m,gospa_loc,gospa_missed,"
      "gospa_false,track_limit\n";
  auto o = [](const std::optional<double>& v) { return v ? fixed(*v, 4) : std::string(); };
  for (const auto& [key, m] : r.runs) {
    const auto& [t, c, sc, run] = key;
    out += r.trackers[t].name + "," + r.conditions[c].set + "," + r.conditions[c].id + "," + sc +
           "," + std::to_string(run) + "," + o(m.mean_rmse) + "," + o(m.max_rmse) + "," +
           std::to_string(m.expected_targets) + "," + std::to_string(m.untracked.size()) + "," +
           o(m.missed_pct) + "," + fixed(m.id_switches, 0) + "," + fixed(m.false_tracks, 0) + "," +
           fixed(m.gospa, 4) + "," + fixed(m.gospa_loc, 4) + "," + fixed(m.gospa_missed, 4) + "," +
           fixed(m.gospa_false, 4) + "," +
           (m.engine_metrics ? std::to_string(m.track_limit) : std::string()) + "\n";
  }
  return out;
}

void write_file(const std::string& path, const std::string& text) {
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  f.write(text.data(), static_cast<std::streamsize>(text.size()));
  if (!f.good()) {
    throw std::runtime_error("cannot write " + path);
  }
}

}  // namespace
}  // namespace sifted::eval

int main(int argc, char** argv) {
  using namespace sifted::eval;
  try {
    const Options o = parse_args(argc, argv);
    if (o.help) {
      std::fputs(kUsage, stdout);
      return 0;
    }
    std::string timing;
    const Results r = evaluate(o, timing);
    const std::string text = o.markdown ? markdown(o, r) : csv(r);
    if (!o.timing.empty()) {
      write_file(o.timing, timing);
    }
    if (o.out.empty()) {
      std::fwrite(text.data(), 1, text.size(), stdout);
    } else {
      write_file(o.out, text);
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "sifted_montecarlo: %s\n", e.what());
    return 2;
  }
}
