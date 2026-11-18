// sifted_eval: the ground-truth tracking scorecard (docs/evaluation.md).
//
// Runs every scenario found under --scenarios through the engine (direct
// feed, as the regression tests do) and through net::Pipeline exactly as the
// daemon consumes it, under the shared impairment levels, outage cases and
// sensor-fault cases (sifted/eval/conditions.hpp), each over several seeds,
// and writes one table in CSV or Markdown. Scoring is the tests' own
// (sifted/eval/scenario.hpp).
//
// Deterministic: fixed seeds (SplitMix64 inside the impairment model), sorted
// iteration, fixed number formatting and no clock, so two runs with the same
// arguments write byte-identical files.

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "sifted/eval/conditions.hpp"
#include "sifted/eval/consistency.hpp"
#include "sifted/eval/scenario.hpp"
#include "sifted/net/impairment.hpp"
#include "sifted/net/pipeline_config.hpp"

namespace sifted::eval {
namespace {

namespace fs = std::filesystem;

constexpr const char* kUsage =
    "usage: sifted_eval [options]\n"
    "  --scenarios PATH  a scenario directory, or a directory of them; repeatable\n"
    "                    (default: the repository's tests/fixtures/scenarios)\n"
    "  --seeds LIST      impairment seeds, comma-separated (default 7,8,9)\n"
    "  --levels LIST     impairment levels (default: all of clean,mild,moderate,severe)\n"
    "  --window LIST     reorder windows [s] for the pipeline runs (default 0.5)\n"
    "  --format csv|md   output format (default md)\n"
    "  --out FILE        write to FILE instead of standard output\n";

// ---- options ------------------------------------------------------------------

struct Options {
  std::vector<std::string> scenario_paths;
  std::vector<std::uint64_t> seeds{7, 8, 9};
  std::vector<std::string> levels;
  std::vector<double> windows{kDaemonReorderWindowS};
  bool markdown = true;
  std::string out;
  std::vector<std::string> args;  // as given, echoed in the report
  bool help = false;
};

std::vector<std::string> split(std::string_view s) {
  std::vector<std::string> out;
  while (true) {
    const std::size_t c = s.find(',');
    out.emplace_back(s.substr(0, c));
    if (c == std::string_view::npos) {
      return out;
    }
    s.remove_prefix(c + 1);
  }
}

template <class T>
T number(const std::string& s, const char* what) {
  T v{};
  const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (s.empty() || ec != std::errc{} || p != s.data() + s.size()) {
    throw std::invalid_argument(std::string("bad ") + what + ": '" + s + "'");
  }
  return v;
}

template <class T>
void sort_unique(std::vector<T>& v) {
  std::sort(v.begin(), v.end());
  v.erase(std::unique(v.begin(), v.end()), v.end());
}

Options parse_args(int argc, char** argv) {
  Options o;
  for (const ImpairmentLevel& l : impairment_levels()) {
    o.levels.push_back(l.name);
  }
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      o.help = true;
      return o;
    }
    if (a != "--scenarios" && a != "--seeds" && a != "--levels" && a != "--window" &&
        a != "--format" && a != "--out") {
      throw std::invalid_argument("unknown option " + a);
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value for " + a);
    }
    const std::string v = argv[++i];
    o.args.push_back(a);
    o.args.push_back(v);
    if (a == "--scenarios") {
      o.scenario_paths.push_back(v);
    } else if (a == "--seeds") {
      o.seeds.clear();
      for (const std::string& s : split(v)) {
        o.seeds.push_back(number<std::uint64_t>(s, "seed"));
      }
    } else if (a == "--levels") {
      const std::vector<std::string> want = split(v);
      for (const std::string& l : want) {
        (void)impairment_level(l);  // throws for an unknown level
      }
      o.levels.clear();  // keep the definition order
      for (const ImpairmentLevel& l : impairment_levels()) {
        if (std::find(want.begin(), want.end(), l.name) != want.end()) {
          o.levels.push_back(l.name);
        }
      }
    } else if (a == "--window") {
      o.windows.clear();
      for (const std::string& s : split(v)) {
        const double w = number<double>(s, "window");
        if (!(w >= 0.0 && w <= 3600.0)) {
          throw std::invalid_argument("window out of range: " + s);
        }
        o.windows.push_back(w);
      }
    } else if (a == "--format") {
      if (v != "csv" && v != "md") {
        throw std::invalid_argument("--format must be csv or md");
      }
      o.markdown = v == "md";
    } else {
      o.out = v;
    }
  }
  sort_unique(o.seeds);
  sort_unique(o.windows);
  if (o.scenario_paths.empty()) {
    o.scenario_paths.push_back(SIFTED_SCENARIO_DIR);
  }
  return o;
}

// ---- scenarios ------------------------------------------------------------------

struct NamedScenario {
  std::string name;
  Scenario sc;
};

std::vector<NamedScenario> discover(const std::vector<std::string>& paths) {
  std::map<std::string, std::string> dirs;  // name -> directory, sorted by name
  auto add = [&](const fs::path& dir) {
    const std::string name = dir.filename().string();
    if (!dirs.emplace(name, dir.string()).second) {
      throw std::invalid_argument("two scenarios named " + name);
    }
  };
  for (const std::string& p : paths) {
    fs::path path(p);
    while (!path.empty() && !path.has_filename()) {
      path = path.parent_path();  // "dir/" -> "dir"
    }
    if (is_scenario_dir(path.string())) {
      add(path);
      continue;
    }
    if (!fs::is_directory(path)) {
      throw std::invalid_argument(p + ": not a directory");
    }
    std::size_t found = 0;
    for (const fs::directory_entry& e : fs::directory_iterator(path)) {
      if (e.is_directory() && is_scenario_dir(e.path().string())) {
        add(e.path());
        ++found;
      }
    }
    if (found == 0) {
      throw std::invalid_argument(p + ": no scenario (sensors.cfg, frames.hex, truth.csv) found");
    }
  }
  std::vector<NamedScenario> out;
  for (const auto& [name, dir] : dirs) {
    out.push_back({name, load_scenario_dir(dir)});
    if (out.back().sc.truth.empty()) {
      throw std::invalid_argument(dir + ": truth.csv has no rows");
    }
  }
  return out;
}

// ---- conditions -------------------------------------------------------------------

struct Condition {
  std::string name;
  bool direct = false;             // engine fed scan by scan (no network, no pipeline)
  bool drop_bearing_only = false;  // direct, with every bearing-only measurement removed
  net::LinkProfile link{};
  std::optional<std::int64_t> outage_s{};  // length of an outage of every link
  const FaultCase* fault = nullptr;
  // A link with nothing random in it gives the same arrivals for every seed
  // (no loss, no jitter, no duplicates): it is run once.
  bool seeded() const { return !direct && !(link == net::LinkProfile{}); }
};

std::vector<Condition> conditions(const Options& o) {
  std::vector<Condition> c;
  c.push_back({.name = "direct", .direct = true});
  c.push_back({.name = "direct without bearing-only", .direct = true, .drop_bearing_only = true});
  for (const std::string& level : o.levels) {
    const Condition base{.name = level, .link = impairment_level(level)};
    c.push_back(base);
    if (level == kOutageLevel) {
      for (const std::int64_t len : kOutageLengthsS) {
        Condition x = base;
        x.name = level + " + " + std::to_string(len) + " s outage";
        x.outage_s = len;
        c.push_back(x);
      }
    }
    if (level == kFaultLevel) {
      for (const FaultCase& f : fault_cases()) {
        Condition x = base;
        x.name = level + " + " + f.name;
        x.fault = &f;
        c.push_back(x);
      }
    }
  }
  return c;
}

bool applies(const Condition& c, const Scenario& sc) {
  if (c.drop_bearing_only) {
    return std::any_of(sc.measurements.begin(), sc.measurements.end(),
                       [](const Measurement& m) { return m.kind == SensorKind::BearingOnly; });
  }
  if (c.fault != nullptr) {
    return first_of_kind(sc.sensors, c.fault->kind).has_value();
  }
  return true;
}

// ---- running ----------------------------------------------------------------------

struct Run {
  Score score;
  std::uint64_t duplicates = 0;
  std::uint64_t reorder_late = 0;
  std::optional<Recovery> recovery;
};

Run run_one(const Scenario& sc, const Condition& c, double window_s, std::uint64_t seed) {
  Run r;
  if (c.direct) {
    std::vector<Measurement> ms = sc.measurements;
    if (c.drop_bearing_only) {
      std::erase_if(ms, [](const Measurement& m) { return m.kind == SensorKind::BearingOnly; });
    }
    r.score = run_engine(sc, ms);
    return r;
  }
  const std::int64_t t0 = sc.truth.front().time_ns;
  std::vector<Measurement> ms = sc.measurements;
  std::optional<std::pair<std::int64_t, std::int64_t>> window;  // recovery measured after it
  if (c.fault != nullptr) {
    const std::int64_t a = t0 + c.fault->start_s * kNsPerS;
    const std::int64_t b = a + c.fault->length_s * kNsPerS;
    ms = c.fault->apply(ms, *first_of_kind(sc.sensors, c.fault->kind), a, b);
    if (c.fault->recovery) {
      window = {a, b};
    }
  }
  net::ImpairmentConfig ic;
  ic.seed = seed;
  ic.global = c.link;
  if (c.outage_s) {
    const std::int64_t a = t0 + kOutageStartS * kNsPerS;
    window = {a, a + *c.outage_s * kNsPerS};
    ic.outages.push_back({window->first, window->second});
  }
  net::PipelineConfig pc;
  pc.reorder_window_s = window_s;
  const PipelineRun p = run_pipeline(sc, net::impair(sent_frames(ms), ic), pc);
  r.score = p.score;
  r.duplicates = p.duplicates;
  r.reorder_late = p.reorder_late;
  if (window) {
    r.recovery = recovery_after(r.score, window->first, window->second);
  }
  return r;
}

// Average over runs of each run's own ratio n / d, in percent (runs with d = 0
// are left out). This is how docs/design.md section 9 aggregates: every run
// weighs the same, whatever its length.
struct RatioMean {
  double sum = 0.0;
  std::size_t n = 0;
  void add(std::uint64_t num, std::uint64_t den) {
    if (den > 0) {
      sum += 100.0 * static_cast<double>(num) / static_cast<double>(den);
      ++n;
    }
  }
  void merge(const RatioMean& o) {
    sum += o.sum;
    n += o.n;
  }
  std::optional<double> value() const {
    return n ? std::optional(sum / static_cast<double>(n)) : std::nullopt;
  }
};

// One row: a (scenario, condition, window) over its seeds, or the merge of
// such rows over every scenario.
struct Cell {
  std::size_t runs = 0;
  std::size_t truth_targets = 0;
  std::size_t targets_scored = 0;  // fewest over the runs; summed when merged
  std::size_t rmse_runs = 0;       // runs with at least one target scored
  double sum_mean_rmse = 0.0;
  double worst_rmse = 0.0;
  std::map<int, std::pair<double, std::size_t>> target_rmse;  // sum, runs
  std::size_t missed = 0;
  std::size_t eligible = 0;
  std::size_t id_switches = 0;
  std::size_t false_tracks = 0;
  std::uint64_t measurements = 0;
  std::uint64_t out_of_order = 0;
  std::uint64_t duplicates = 0;
  std::uint64_t reorder_late = 0;
  RatioMean missed_pct;        // of eligible target-instants
  RatioMean out_of_order_pct;  // of measurements offered to the engine
  bool has_recovery = false;
  std::size_t recovered = 0;      // runs that recovered
  std::size_t not_recovered = 0;  // runs with targets to recover that never did
  double sum_recovery_s = 0.0;
  double worst_recovery_s = 0.0;
  std::size_t survived = 0;
  std::size_t considered = 0;
  NeesByRegime nees;
  std::map<SensorKind, ChiSquareStat> nis;

  void add(const Run& r) {
    const Score& s = r.score;
    targets_scored = runs == 0 ? s.rmse.size() : std::min(targets_scored, s.rmse.size());
    ++runs;
    if (!s.rmse.empty()) {
      ++rmse_runs;
      sum_mean_rmse += s.mean_rmse;
      worst_rmse = std::max(worst_rmse, s.max_rmse);
    }
    for (const auto& [tgt, v] : s.rmse) {
      target_rmse[tgt].first += v;
      ++target_rmse[tgt].second;
    }
    missed += s.missed;
    eligible += s.eligible;
    id_switches += s.id_switches;
    false_tracks += s.false_tracks;
    measurements += s.measurements;
    out_of_order += s.out_of_order;
    duplicates += r.duplicates;
    reorder_late += r.reorder_late;
    missed_pct.add(s.missed, s.eligible);
    out_of_order_pct.add(s.out_of_order, s.measurements);
    if (r.recovery) {
      has_recovery = true;
      const Recovery& rec = *r.recovery;
      if (rec.targets > 0 && rec.recovery_s < 0.0) {
        ++not_recovered;
      } else if (rec.targets > 0) {
        ++recovered;
        sum_recovery_s += rec.recovery_s;
        worst_recovery_s = std::max(worst_recovery_s, rec.recovery_s);
        survived += rec.survived;
        considered += rec.targets;
      }
    }
    merge_nees(s.nees, s.nis);
  }

  void merge(const Cell& o) {
    runs += o.runs;
    truth_targets += o.truth_targets;
    targets_scored += o.targets_scored;
    rmse_runs += o.rmse_runs;
    sum_mean_rmse += o.sum_mean_rmse;
    worst_rmse = std::max(worst_rmse, o.worst_rmse);
    missed += o.missed;
    eligible += o.eligible;
    id_switches += o.id_switches;
    false_tracks += o.false_tracks;
    measurements += o.measurements;
    out_of_order += o.out_of_order;
    duplicates += o.duplicates;
    reorder_late += o.reorder_late;
    missed_pct.merge(o.missed_pct);
    out_of_order_pct.merge(o.out_of_order_pct);
    has_recovery = has_recovery || o.has_recovery;
    recovered += o.recovered;
    not_recovered += o.not_recovered;
    sum_recovery_s += o.sum_recovery_s;
    worst_recovery_s = std::max(worst_recovery_s, o.worst_recovery_s);
    survived += o.survived;
    considered += o.considered;
    merge_nees(o.nees, o.nis);
  }

  void merge_nees(const NeesByRegime& n, const std::map<SensorKind, ChiSquareStat>& k) {
    for (auto [mine, theirs] :
         {std::pair{&nees.updated, &n.updated}, std::pair{&nees.extrapolated, &n.extrapolated}}) {
      mine->full.merge(theirs->full);
      mine->position.merge(theirs->position);
    }
    for (const auto& [kind, st] : k) {
      nis[kind].merge(st);
    }
  }

  std::optional<double> mean_rmse() const {
    return rmse_runs ? std::optional(sum_mean_rmse / static_cast<double>(rmse_runs)) : std::nullopt;
  }
  std::optional<double> mean_recovery() const {
    return recovered ? std::optional(sum_recovery_s / static_cast<double>(recovered))
                     : std::nullopt;
  }
};

// Row key: scenario index (npos: every scenario), condition index, window
// index (-1: direct conditions, which have no window).
using Key = std::tuple<std::size_t, std::size_t, int>;
constexpr std::size_t kAll = static_cast<std::size_t>(-1);

struct Results {
  std::vector<std::string> scenarios;
  std::vector<Condition> conditions;
  std::vector<double> windows;
  std::vector<std::uint64_t> seeds;
  std::map<Key, Cell> cells;  // per scenario
  std::map<Key, Cell> all;    // merged over scenarios (scenario index kAll)
};

Results evaluate(const Options& o) {
  Results res;
  res.conditions = conditions(o);
  res.windows = o.windows;
  res.seeds = o.seeds;
  const std::vector<NamedScenario> scenarios = discover(o.scenario_paths);
  for (std::size_t si = 0; si < scenarios.size(); ++si) {
    const Scenario& sc = scenarios[si].sc;
    res.scenarios.push_back(scenarios[si].name);
    std::map<int, bool> targets;
    for (const TruthRow& t : sc.truth) {
      targets[t.target] = true;
    }
    for (std::size_t ci = 0; ci < res.conditions.size(); ++ci) {
      const Condition& c = res.conditions[ci];
      if (!applies(c, sc)) {
        continue;
      }
      const int windows = c.direct ? 1 : static_cast<int>(o.windows.size());
      for (int wi = 0; wi < windows; ++wi) {
        Cell& cell = res.cells[{si, ci, c.direct ? -1 : wi}];
        cell.truth_targets = targets.size();
        const double w = c.direct ? 0.0 : o.windows[static_cast<std::size_t>(wi)];
        if (c.seeded()) {
          for (const std::uint64_t seed : o.seeds) {
            cell.add(run_one(sc, c, w, seed));
          }
        } else {
          cell.add(run_one(sc, c, w, o.seeds.empty() ? 1 : o.seeds.front()));
        }
      }
    }
  }
  for (const auto& [key, cell] : res.cells) {
    res.all[{kAll, std::get<1>(key), std::get<2>(key)}].merge(cell);
  }
  return res;
}

// ---- formatting -------------------------------------------------------------------

std::string fixed(double v, int precision) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", precision, v);
  std::string s = buf;
  if (s.find_first_not_of("-0.") == std::string::npos && s.front() == '-') {
    s.erase(0, 1);  // "-0.00" -> "0.00"
  }
  return s;
}

std::string opt(const std::optional<double>& v, int precision, const char* none = "") {
  return v ? fixed(*v, precision) : none;
}

std::optional<double> average(const ChiSquareStat& s) {
  return s.n ? std::optional(s.average()) : std::nullopt;
}

const char* kind_name(SensorKind k) {
  switch (k) {
    case SensorKind::RangeBearing:
      return "range-bearing";
    case SensorKind::BearingOnly:
      return "bearing-only";
    case SensorKind::GeoPosition:
      return "geo-position";
  }
  return "unknown";
}
constexpr SensorKind kKinds[] = {SensorKind::RangeBearing, SensorKind::BearingOnly,
                                 SensorKind::GeoPosition};

std::string window_label(const Results& r, int wi) {
  return wi < 0 ? "" : fixed(r.windows[static_cast<std::size_t>(wi)], 2);
}

std::string per_target(const Cell& c) {
  std::string s;
  for (const auto& [tgt, sum] : c.target_rmse) {
    s += (s.empty() ? "" : " ") + std::to_string(tgt) + ":" +
         fixed(sum.first / static_cast<double>(sum.second), 2);
  }
  return s;
}

std::string csv(const Results& r) {
  std::string out =
      "scenario,condition,window_s,runs,truth_targets,targets_scored,mean_rmse_m,worst_rmse_m,"
      "per_target_rmse_m,missed,eligible,missed_pct,id_switches,false_tracks,measurements,"
      "out_of_order,out_of_order_pct,duplicates,reorder_late,recovered_runs,not_recovered_runs,"
      "recovery_mean_s,recovery_worst_s,survived,considered,nees4_updated,nees4_extrapolated,"
      "nees2_updated,nees2_extrapolated";
  for (const SensorKind k : kKinds) {
    std::string n = kind_name(k);
    std::replace(n.begin(), n.end(), '-', '_');
    out += ",nis_" + n;
  }
  out += "\n";
  auto row = [&](const std::string& scenario, const Key& key, const Cell& c) {
    const Condition& cond = r.conditions[std::get<1>(key)];
    std::vector<std::string> f = {
        scenario,
        cond.name,
        window_label(r, std::get<2>(key)),
        std::to_string(c.runs),
        std::to_string(c.truth_targets),
        std::to_string(c.targets_scored),
        opt(c.mean_rmse(), 2),
        c.rmse_runs ? fixed(c.worst_rmse, 2) : "",
        per_target(c),
        std::to_string(c.missed),
        std::to_string(c.eligible),
        opt(c.missed_pct.value(), 2),
        std::to_string(c.id_switches),
        std::to_string(c.false_tracks),
        std::to_string(c.measurements),
        std::to_string(c.out_of_order),
        opt(c.out_of_order_pct.value(), 2),
        std::to_string(c.duplicates),
        std::to_string(c.reorder_late),
        c.has_recovery ? std::to_string(c.recovered) : "",
        c.has_recovery ? std::to_string(c.not_recovered) : "",
        opt(c.mean_recovery(), 1),
        c.recovered ? fixed(c.worst_recovery_s, 1) : "",
        c.has_recovery ? std::to_string(c.survived) : "",
        c.has_recovery ? std::to_string(c.considered) : "",
        opt(average(c.nees.updated.full), 2),
        opt(average(c.nees.extrapolated.full), 2),
        opt(average(c.nees.updated.position), 2),
        opt(average(c.nees.extrapolated.position), 2),
    };
    for (const SensorKind k : kKinds) {
      const auto it = c.nis.find(k);
      f.push_back(it == c.nis.end() ? "" : opt(average(it->second), 2));
    }
    for (std::size_t i = 0; i < f.size(); ++i) {
      out += (i ? "," : "") + f[i];
    }
    out += "\n";
  };
  for (const auto& [key, cell] : r.cells) {
    row(r.scenarios[std::get<0>(key)], key, cell);
  }
  for (const auto& [key, cell] : r.all) {
    row("ALL", key, cell);
  }
  return out;
}

// ---- markdown ---------------------------------------------------------------------

std::string millis(std::int64_t ns) { return fixed(static_cast<double>(ns) * 1e-6, 0) + " ms"; }

std::string describe(const net::LinkProfile& p) {
  if (p == net::LinkProfile{}) {
    return "no loss, delay, jitter or duplication";
  }
  std::string s = fixed(100.0 * p.loss, 0) + "% loss";
  if (p.p_good_to_bad > 0.0) {
    s += " + Gilbert-Elliott bursts (good->bad " + fixed(p.p_good_to_bad, 2) + ", bad->good " +
         fixed(p.p_bad_to_good, 2) + ", " + fixed(100.0 * p.loss_bad, 0) + "% loss when bad)";
  }
  s += ", delay " + millis(p.delay_ns) + " + U(0, " + millis(p.jitter_ns) + "), " +
       fixed(100.0 * p.duplicate, 0) + "% duplicated";
  return s;
}

std::string join_seeds(const std::vector<std::uint64_t>& seeds) {
  std::string s;
  for (const std::uint64_t v : seeds) {
    s += (s.empty() ? "" : ", ") + std::to_string(v);
  }
  return s;
}

std::string command_line(const Options& o) {
  std::string s = "sifted_eval";
  for (const std::string& a : o.args) {
    s += " " + a;
  }
  return s;
}

std::string markdown(const Options& o, const Results& r) {
  const std::string dash = "–";
  auto md_opt = [&](const std::optional<double>& v, int p) { return opt(v, p, "–"); };
  auto cond_label = [&](const Key& k) {
    const std::string w = window_label(r, std::get<2>(k));
    return r.conditions[std::get<1>(k)].name + " | " + (w.empty() ? dash : w);
  };
  auto recovery = [&](const Cell& c) {
    if (!c.has_recovery) {
      return dash;
    }
    std::string s =
        c.recovered ? fixed(*c.mean_recovery(), 1) + " / " + fixed(c.worst_recovery_s, 1) : dash;
    if (c.not_recovered) {
      s += " (" + std::to_string(c.not_recovered) + " never)";
    }
    return s;
  };

  std::string out;
  out += "# Evaluation scorecard\n\n";
  out += "<!-- Generated by sifted_eval (eval/sifted_eval.cpp). Do not edit by hand. -->\n\n";
  out +=
      "How accurately and robustly Sifted tracks, scored against simulator ground truth over "
      "every scenario fixture, under network impairment and sensor faults. All numbers on this "
      "page come from one command (run from the repository root):\n\n";
  out += "```bash\ncmake --preset release && cmake --build --preset release\n";
  out += "build/release/eval/" + command_line(o) + "\n```\n\n";
  out +=
      "The tool, the regression tests and the degraded-network and fault tests share one "
      "implementation of loading, scoring and the conditions (`eval/sifted/eval/`); a test "
      "(`Eval.*` in `tests/eval_test.cpp`) checks that the tool's direct-feed numbers equal the "
      "regression baselines. Accuracy does not depend on CPU speed, load or timing: the inputs "
      "are recorded fixtures, time is an input, the impairment model is seeded, and the engine "
      "is a deterministic function of its ordered input (docs/design.md section 3). Running the "
      "command twice gives a byte-identical file, and any machine with the same compiler and "
      "maths library gives the same numbers.\n\n";

  out += "Scenarios: ";
  for (std::size_t i = 0; i < r.scenarios.size(); ++i) {
    out += (i ? ", " : "") + std::string("`") + r.scenarios[i] + "`";
  }
  out += ". Seeds: " + join_seeds(r.seeds) + ". Reorder windows: ";
  for (std::size_t i = 0; i < r.windows.size(); ++i) {
    out += (i ? ", " : "") + fixed(r.windows[i], 2) + " s";
  }
  out += " (the daemon uses " + fixed(kDaemonReorderWindowS, 2) + " s).\n\n";

  // Headline.
  out += "## Summary over all scenarios\n\n";
  out +=
      "Each row merges every scenario and seed of one condition (runs = scenarios x seeds). Mean "
      "RMSE is the average over runs of each run's mean over targets; worst RMSE is the largest "
      "single-target RMSE of any run; missed % and out-of-order % average each run's own ratio "
      "(missed target-instants / eligible, out-of-order drops / measurements); ID switches and "
      "false tracks are totals.\n\n";
  out +=
      "| condition | window s | runs | mean RMSE m | worst RMSE m | missed % | ID sw. | false | "
      "out-of-order % | recovery mean / worst s |\n";
  out += "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for (const auto& [key, c] : r.all) {
    out += "| " + cond_label(key) + " | " + std::to_string(c.runs) + " | " +
           md_opt(c.mean_rmse(), 2) + " | " + (c.rmse_runs ? fixed(c.worst_rmse, 2) : dash) +
           " | " + md_opt(c.missed_pct.value(), 2) + " | " + std::to_string(c.id_switches) + " | " +
           std::to_string(c.false_tracks) + " | " + md_opt(c.out_of_order_pct.value(), 2) + " | " +
           recovery(c) + " |\n";
  }

  // Direct feed reference.
  out += "\n## Direct engine feed (regression reference)\n\n";
  out +=
      "The frames fed straight into the engine scan by scan, as the regression tests do; no "
      "network, no pipeline. \"Without bearing-only\" is the same input with every bearing-only "
      "measurement removed: fusion must lower the RMSE.\n\n";
  out +=
      "| scenario | per-target RMSE m | mean RMSE m | missed | ID sw. | false | mean RMSE without "
      "bearing-only m |\n";
  out += "|---|---|---:|---:|---:|---:|---:|\n";
  for (std::size_t si = 0; si < r.scenarios.size(); ++si) {
    const auto it = r.cells.find({si, 0, -1});
    if (it == r.cells.end()) {
      continue;
    }
    const Cell& c = it->second;
    const auto nb = r.cells.find({si, 1, -1});
    out += "| " + r.scenarios[si] + " | " + per_target(c) + " | " + md_opt(c.mean_rmse(), 2) +
           " | " + std::to_string(c.missed) + "/" + std::to_string(c.eligible) + " | " +
           std::to_string(c.id_switches) + " | " + std::to_string(c.false_tracks) + " | " +
           (nb == r.cells.end() ? dash : md_opt(nb->second.mean_rmse(), 2)) + " |\n";
  }

  // Per scenario.
  out += "\n## Per scenario and condition\n\n";
  out +=
      "Targets: scored / present in truth (fewest over the seeds). Missed: target-instants, "
      "summed over seeds. Measurements: offered to the engine; out-of-order: of those, dropped "
      "by the engine; duplicates: suppressed by the pipeline; late: frames that arrived after "
      "the reorder stage had released a later time.\n\n";
  out +=
      "| scenario | condition | window s | runs | targets | mean RMSE m | worst RMSE m | missed | "
      "ID sw. | false | measurements | out-of-order | duplicates | late | recovery mean / worst "
      "s | survived |\n";
  out += "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for (const auto& [key, c] : r.cells) {
    const bool direct = r.conditions[std::get<1>(key)].direct;
    out += "| " + r.scenarios[std::get<0>(key)] + " | " + cond_label(key) + " | " +
           std::to_string(c.runs) + " | " + std::to_string(c.targets_scored) + "/" +
           std::to_string(c.truth_targets) + " | " + md_opt(c.mean_rmse(), 2) + " | " +
           (c.rmse_runs ? fixed(c.worst_rmse, 2) : dash) + " | " + std::to_string(c.missed) + "/" +
           std::to_string(c.eligible) + " | " + std::to_string(c.id_switches) + " | " +
           std::to_string(c.false_tracks) + " | " + std::to_string(c.measurements) + " | " +
           std::to_string(c.out_of_order) + " | " + (direct ? dash : std::to_string(c.duplicates)) +
           " | " + (direct ? dash : std::to_string(c.reorder_late)) + " | " + recovery(c) + " | " +
           (c.has_recovery && c.considered
                ? std::to_string(c.survived) + "/" + std::to_string(c.considered)
                : dash) +
           " |\n";
  }

  // Consistency.
  std::vector<SensorKind> kinds;
  for (const SensorKind k : kKinds) {
    for (const auto& [key, c] : r.cells) {
      if (std::get<1>(key) == 0 && c.nis.contains(k)) {
        kinds.push_back(k);
        break;
      }
    }
  }
  out += "\n## Filter consistency (direct engine feed)\n\n";
  out +=
      "Average NEES (4-dof full state, 2-dof position) over the RMSE's samples, split into "
      "estimates at their own update instant (\"upd.\") and extrapolated or coasting ones "
      "(\"extr.\"), and average NIS per sensor kind; a consistent filter averages the degrees of "
      "freedom. The bracket is the 95% interval for the updated NEES4 under independence, which "
      "these time-correlated samples do not satisfy: a guide, not a test (docs/design.md section "
      "7).\n\n";
  out += "| scenario | NEES4 upd. (95%) | NEES4 extr. | NEES2 upd. | NEES2 extr. |";
  std::string rule = "|---|---:|---:|---:|---:|";
  for (const SensorKind k : kinds) {
    out += std::string(" NIS ") + kind_name(k) + " |";
    rule += "---:|";
  }
  out += "\n" + rule + "\n";
  for (std::size_t si = 0; si < r.scenarios.size(); ++si) {
    const auto it = r.cells.find({si, 0, -1});
    if (it == r.cells.end()) {
      continue;
    }
    const Cell& c = it->second;
    const ChiSquareStat& u = c.nees.updated.full;
    std::string nees4 = md_opt(average(u), 2);
    if (u.n) {
      const Interval iv = u.interval();
      nees4 += " [" + fixed(iv.lo, 2) + ", " + fixed(iv.hi, 2) + "]";
    }
    out += "| " + r.scenarios[si] + " | " + nees4 + " | " +
           md_opt(average(c.nees.extrapolated.full), 2) + " | " +
           md_opt(average(c.nees.updated.position), 2) + " | " +
           md_opt(average(c.nees.extrapolated.position), 2) + " |";
    for (const SensorKind k : kinds) {
      const auto n = c.nis.find(k);
      out += " " + (n == c.nis.end() ? dash : md_opt(average(n->second), 2)) + " |";
    }
    out += "\n";
  }

  // Definitions.
  out += "\n## Conditions\n\n";
  out +=
      "- **direct**: the scenario's frames, decoded by the real decoder, fed to the engine scan "
      "by scan (`run_engine`). The regression tests' path.\n"
      "- **pipeline conditions** (every other row): frames sent at their measurement time "
      "through the seeded impairment model (ADR 0008), then fed to `net::Pipeline` exactly as "
      "the daemon consumes them: arrival order, receive time = arrival time, a quiet tick after " +
      fixed(static_cast<double>(kDaemonTickNs) * 1e-6, 0) +
      " ms (`run_pipeline`). Truth instant T is scored on the tracks the pipeline holds at T, "
      "extrapolated to T, so "
      "even a clean network scores slightly worse than the direct feed: the scan stamped T is "
      "still being assembled.\n";
  for (const ImpairmentLevel& l : impairment_levels()) {
    out += "- **" + l.name + "**: " + describe(l.profile) + ".\n";
  }
  out += "- **" + std::string(kOutageLevel) + " + N s outage**: every link down for N s, from " +
         std::to_string(kOutageStartS) + " s after the first truth instant, on the " +
         std::string(kOutageLevel) + " level.\n";
  for (const FaultCase& f : fault_cases()) {
    out += "- **" + std::string(kFaultLevel) + " + " + f.name + "**: the first " +
           kind_name(f.kind) + " sensor of the table " + f.description +
           "; every other sensor untouched (`eval/sifted/eval/conditions.cpp`).\n";
  }
  out +=
      "\nA link with nothing random in it gives the same arrivals for every seed, so those "
      "conditions run once per window. Conditions that need a sensor kind a scenario lacks are "
      "left out for it.\n";

  out += "\n## Metric definitions\n\n";
  out +=
      "CLEAR-MOT style (Bernardin and Stiefelhagen, 2008); the authoritative definitions are in "
      "`eval/sifted/eval/scenario.hpp`. At each truth time T, once the tracker has consumed its "
      "input up to T:\n\n"
      "- **Reported tracks** are the confirmed and coasting tracks, extrapolated to T. They are "
      "matched one-to-one to the truth targets present at T: a target's previous match is kept "
      "while that track is still reported and within " +
      fixed(kMatchDistanceM, 0) +
      " m; the rest are matched by a minimum-total-distance assignment with pairs further apart "
      "than " +
      fixed(kMatchDistanceM, 0) +
      " m forbidden. The matcher knows nothing about which track \"should\" be which target.\n"
      "- **RMSE** (position, per target): over covered, matched instants at least " +
      fixed(kRmseBurnInS, 0) +
      " s after the target first enters coverage. Mean RMSE averages the targets; worst RMSE is "
      "the largest.\n"
      "- **Missed**: covered targets unmatched at least " +
      fixed(kMissBurnInS, 0) +
      " s after they first enter coverage, out of all such (eligible) target-instants.\n"
      "- **ID switches**: a target matched to a different track than the last time it was "
      "matched.\n"
      "- **False tracks**: reported tracks matched to any target (covered or not) in fewer than "
      "half of the instants they were reported.\n"
      "- **Recovery** after an outage or fault window [start, end): the targets covered and "
      "matched at the last truth instant before start and still covered after end; a target has "
      "recovered at the first instant >= end at which it is matched to a track updated after "
      "end (extrapolation through the gap does not count). Recovery time runs from end until all "
      "have recovered; \"survived\" counts targets still on the track they had before. Mean and "
      "worst are over the runs that recovered; \"never\" counts runs that did not within the "
      "scenario.\n"
      "- **NEES** e^T P^-1 e of the estimate against truth and **NIS** y^T S^-1 y (the engine's "
      "own gating distance of every associated update on a matched track), as in "
      "`eval/sifted/eval/consistency.hpp`.\n";
  return out;
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
    const Results r = evaluate(o);
    const std::string text = o.markdown ? markdown(o, r) : csv(r);
    if (o.out.empty()) {
      std::fwrite(text.data(), 1, text.size(), stdout);
      return 0;
    }
    std::ofstream f(o.out, std::ios::binary | std::ios::trunc);
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    if (!f.good()) {
      std::fprintf(stderr, "sifted_eval: cannot write %s\n", o.out.c_str());
      return 1;
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "sifted_eval: %s\n%s", e.what(), kUsage);
    return 2;
  }
}
