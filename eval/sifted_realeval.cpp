// sifted_realeval: real-data evaluation A (docs/real-data-evaluation.md).
//
// Runs every withheld-report run directory found under --runs (written by
// `python3 -m sifted_sim.realeval`, layout in sifted/eval/realdata.hpp)
// through the engine with the given configuration, and writes the merged
// scores as plain "key values..." lines that the Python report generator
// renders. Scoring is sifted_eval_support's (score_holdout); this tool only
// collects and formats. Developer tooling like sifted_eval: the app preset
// does not build it.
//
// Deterministic: runs are read in name order, every number is printed with
// fixed formatting, and nothing depends on paths, time or the machine.

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "sifted/eval/realdata.hpp"

namespace sifted::eval {
namespace {

namespace fs = std::filesystem;

constexpr const char* kUsage =
    "usage: sifted_realeval --runs PATH [--runs PATH ...] [options]\n"
    "  --runs PATH       a withheld-report run directory, or a directory of them\n"
    "  --bins LIST       horizon bin edges [s], ascending (default 0,2,5,10,20,40,80,160)\n"
    "  --out FILE        write to FILE instead of standard output\n"
    "engine (as sifted_fusiond; defaults are EngineConfig's):\n"
    "  --accel-psd Q  --initial-velocity-sigma V  --gate-probability P  --confirm-hits N\n"
    "  --tentative-timeout S  --coast-after S  --drop-after S  --max-position-sigma M\n";

struct Options {
  std::vector<std::string> runs;
  std::vector<double> bins{0, 2, 5, 10, 20, 40, 80, 160};
  EngineConfig engine;
  std::string out;
  bool help = false;
};

double number(const std::string& s, const char* what) {
  double v = 0.0;
  const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (s.empty() || ec != std::errc{} || p != s.data() + s.size() || !std::isfinite(v)) {
    throw std::invalid_argument(std::string("bad ") + what + ": '" + s + "'");
  }
  return v;
}

Options parse_args(int argc, char** argv) {
  Options o;
  std::map<std::string, double*> tuning = {
      {"--accel-psd", &o.engine.accel_psd},
      {"--initial-velocity-sigma", &o.engine.initial_velocity_sigma},
      {"--gate-probability", &o.engine.association.gate_probability},
      {"--tentative-timeout", &o.engine.lifecycle.tentative_timeout_s},
      {"--coast-after", &o.engine.lifecycle.coast_after_s},
      {"--drop-after", &o.engine.lifecycle.drop_after_s},
      {"--max-position-sigma", &o.engine.lifecycle.max_position_sigma_m},
  };
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      o.help = true;
      return o;
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value for " + a);
    }
    const std::string v = argv[++i];
    if (a == "--runs") {
      o.runs.push_back(v);
    } else if (a == "--out") {
      o.out = v;
    } else if (a == "--bins") {
      o.bins.clear();
      std::size_t start = 0;
      while (true) {
        const std::size_t c = v.find(',', start);
        o.bins.push_back(number(v.substr(start, c - start), "bin edge"));
        if (c == std::string::npos) {
          break;
        }
        start = c + 1;
      }
      for (std::size_t b = 1; b < o.bins.size(); ++b) {
        if (!(o.bins[b] > o.bins[b - 1])) {
          throw std::invalid_argument("--bins must be strictly ascending");
        }
      }
      if (o.bins.size() < 2 || o.bins.front() != 0.0) {
        throw std::invalid_argument("--bins needs at least two edges, starting at 0");
      }
    } else if (a == "--confirm-hits") {
      const double n = number(v, "confirm hits");
      if (!(n >= 1.0 && n <= 1000.0 && n == std::floor(n))) {
        throw std::invalid_argument("bad confirm hits: " + v);
      }
      o.engine.lifecycle.confirm_hits = static_cast<std::uint32_t>(n);
    } else if (const auto t = tuning.find(a); t != tuning.end()) {
      *t->second = number(v, a.c_str());
    } else {
      throw std::invalid_argument("unknown option " + a);
    }
  }
  if (o.runs.empty()) {
    throw std::invalid_argument("--runs is required");
  }
  if (!is_valid(o.engine)) {
    throw std::invalid_argument("engine configuration out of range");
  }
  return o;
}

std::map<std::string, std::string> discover(const std::vector<std::string>& paths) {
  std::map<std::string, std::string> dirs;  // name -> directory, sorted by name
  auto add = [&](const fs::path& dir) {
    if (!dirs.emplace(dir.filename().string(), dir.string()).second) {
      throw std::invalid_argument("two runs named " + dir.filename().string());
    }
  };
  for (const std::string& p : paths) {
    fs::path path(p);
    while (!path.empty() && !path.has_filename()) {
      path = path.parent_path();
    }
    if (is_holdout_dir(path.string())) {
      add(path);
      continue;
    }
    if (!fs::is_directory(path)) {
      throw std::invalid_argument(p + ": not a directory");
    }
    std::size_t found = 0;
    for (const fs::directory_entry& e : fs::directory_iterator(path)) {
      if (e.is_directory() && is_holdout_dir(e.path().string())) {
        add(e.path());
        ++found;
      }
    }
    if (found == 0) {
      throw std::invalid_argument(p + ": no run directory found");
    }
  }
  return dirs;
}

std::string fixed(double v, int precision) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", precision, v);
  std::string s = buf;
  if (s.find_first_not_of("-0.") == std::string::npos && s.front() == '-') {
    s.erase(0, 1);
  }
  return s;
}

std::string q_line(const Quantiles& q) {
  return std::to_string(q.n) + " " + fixed(q.median, 2) + " " + fixed(q.p95, 2) + " " +
         fixed(q.p99, 2) + " " + fixed(q.mean, 2);
}

std::string report(const Options& o) {
  std::string out = "# sifted_realeval: evaluation A scores (sifted/eval/realdata.hpp)\n";
  HoldoutScore all;
  std::string per_run;
  const auto dirs = discover(o.runs);
  for (const auto& [name, dir] : dirs) {
    const HoldoutRun run = load_holdout_dir(dir);
    const HoldoutScore s = score_holdout(run, o.engine);
    std::vector<double> tracker;
    for (const ErrorSample& e : s.errors[static_cast<std::size_t>(Estimator::Tracker)]) {
      tracker.push_back(e.error_m);
    }
    const Quantiles q = quantiles(tracker);
    std::size_t fed = 0;
    for (const auto& [a, c] : s.aircraft) {
      fed += c.fed;
    }
    per_run += "run " + name + " aircraft " + std::to_string(s.aircraft.size()) + " fed " +
               std::to_string(fed) + " withheld " + std::to_string(s.withheld) + " started " +
               std::to_string(s.tracks_started) + " confirmed " +
               std::to_string(s.confirmed_tracks) + " fragments " + std::to_string(s.fragments) +
               " never_confirmed " + std::to_string(s.never_confirmed) + " coverage " +
               std::to_string(s.coverage_eligible()) + " " + std::to_string(s.covered()) +
               " no_track " + std::to_string(s.no_track) + " tracker_median " + fixed(q.median, 2) +
               " max_range_m " + std::to_string(s.max_range_m) + "\n";
    all.merge(s);
  }
  std::size_t fed = 0;
  for (const auto& [a, c] : all.aircraft) {
    fed += c.fed;
  }
  out += "runs " + std::to_string(dirs.size()) + "\n";
  out += "aircraft " + std::to_string(all.aircraft.size()) + "\n";
  out += "fed " + std::to_string(fed) + "\n";
  out += "withheld " + std::to_string(all.withheld) + "\n";
  out += "no_prior_report " + std::to_string(all.no_prior_report) + "\n";
  out += "no_track " + std::to_string(all.no_track) + "\n";
  out += "no_velocity " + std::to_string(all.no_velocity) + "\n";
  out += "max_range_m " + std::to_string(all.max_range_m) + "\n";
  for (const Estimator e : kEstimators) {
    const auto& samples = all.errors[static_cast<std::size_t>(e)];
    std::vector<double> every;
    for (const ErrorSample& x : samples) {
      every.push_back(x.error_m);
    }
    out += std::string("horizon ") + to_string(e) + " all all " + q_line(quantiles(every)) + "\n";
    for (std::size_t b = 0; b < o.bins.size(); ++b) {
      const double lo = o.bins[b];
      const double hi =
          b + 1 < o.bins.size() ? o.bins[b + 1] : std::numeric_limits<double>::infinity();
      std::vector<double> in;
      for (const ErrorSample& x : samples) {
        // First bin [0, e1]; the others (lo, hi]; the last is open above.
        if ((b == 0 ? x.horizon_s >= lo : x.horizon_s > lo) && x.horizon_s <= hi) {
          in.push_back(x.error_m);
        }
      }
      out += std::string("horizon ") + to_string(e) + " " + fixed(lo, 0) + " " +
             (std::isinf(hi) ? "inf" : fixed(hi, 0)) + " " + q_line(quantiles(in)) + "\n";
    }
  }
  const Interval iv = all.nis.n ? all.nis.interval() : Interval{};
  out += "nis " + std::to_string(all.nis.n) + " " + fixed(all.nis.average(), 3) + " " +
         fixed(all.nis.expected(), 3) + " " + fixed(iv.lo, 3) + " " + fixed(iv.hi, 3) + "\n";
  out += "nis_lost " + std::to_string(all.nis_lost) + "\n";
  out += "tracks_started " + std::to_string(all.tracks_started) + "\n";
  out += "confirmed_tracks " + std::to_string(all.confirmed_tracks) + "\n";
  out += "fragments " + std::to_string(all.fragments) + "\n";
  out += "never_confirmed " + std::to_string(all.never_confirmed) + "\n";
  out += "swaps " + std::to_string(all.swaps) + "\n";
  out += "mixed_tracks " + std::to_string(all.mixed_tracks) + "\n";
  out += "inconsistent_tracks " + std::to_string(all.inconsistent_tracks) + "\n";
  out += "coverage " + std::to_string(all.coverage_eligible()) + " " +
         std::to_string(all.covered()) + "\n";
  for (std::size_t k = 0; k < kMeasurementOutcomeCount; ++k) {
    const auto oc = static_cast<MeasurementOutcome>(k);
    const auto it = all.outcomes.find(oc);
    std::string name(to_string(oc));
    std::replace(name.begin(), name.end(), ' ', '_');
    std::replace(name.begin(), name.end(), '-', '_');
    out +=
        "outcome " + name + " " + std::to_string(it == all.outcomes.end() ? 0 : it->second) + "\n";
  }
  return out + per_run;
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
    const std::string text = report(o);
    if (o.out.empty()) {
      std::fwrite(text.data(), 1, text.size(), stdout);
      return 0;
    }
    std::ofstream f(o.out, std::ios::binary | std::ios::trunc);
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    if (!f.good()) {
      std::fprintf(stderr, "sifted_realeval: cannot write %s\n", o.out.c_str());
      return 1;
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "sifted_realeval: %s\n%s", e.what(), kUsage);
    return 2;
  }
}
