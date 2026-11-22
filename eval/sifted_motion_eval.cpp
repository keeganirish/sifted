// sifted_motion_eval: legacy constant-velocity filter against the IMM, side by
// side (ADR 0014, docs/motion-model-evaluation.md).
//
// Runs every scenario found under --scenarios through the engine (direct
// feed, the scoring of sifted/eval/scenario.hpp) once per motion model, and
// writes one Markdown report:
//   * per condition (scenario directories named <condition>_s<seed> are runs
//     of one condition with different noise seeds; others are their own
//     condition): ID switches, missed target-instants, false tracks, position
//     and velocity RMSE, averaged over runs;
//   * filter consistency by flight phase, pooled over every run: 4-dof NEES
//     and NIS per degree of freedom, at instants classified from the truth
//     (never from scenario names):
//       turn     the true velocity changed over the last truth interval by more
//                than kTurnAccel x dt;
//       onset    the first kPhaseS seconds of a turn;
//       exit     the first kPhaseS seconds of straight flight after a turn;
//       straight everything else.
//
// Deterministic: sorted iteration, fixed formatting, no clock or threads.
//
// Typical use (docs/motion-model-evaluation.md lists the exact commands):
//   cd sim && python3 -m sifted_sim.manoeuvres evaluation ../build/release/manoeuvres-eval
//   build/release/eval/sifted_motion_eval --scenarios build/release/manoeuvres-eval
//       --scenarios tests/fixtures/scenarios --out docs/motion-model-evaluation.md

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "sifted/eval/scenario.hpp"

namespace fs = std::filesystem;

namespace sifted::eval {
namespace {

constexpr double kTurnAccel = 0.5;  // m/s^2 (~0.05 g): below every turn evaluated
constexpr double kPhaseS = 10.0;

const char* const kUsage =
    "usage: sifted_motion_eval --scenarios PATH [options]\n"
    "  --scenarios PATH   a scenario directory, or a directory of them; repeatable\n"
    "  --title TEXT       report heading (default: Motion model comparison)\n"
    "  --imm-accel-psd Q0,Q1,Q2  --imm-sojourn T0,T1,T2  --imm-probability-floor P\n"
    "                     IMM parameters (default: EngineConfig defaults)\n"
    "  --out FILE         write to FILE instead of standard output\n";

struct Options {
  std::vector<std::string> paths;
  std::string title = "Motion model comparison";
  EngineConfig imm;
  std::string out;
  std::vector<std::string> args;
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

double number(const std::string& s) {
  double v = 0.0;
  const auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
  if (s.empty() || ec != std::errc{} || p != s.data() + s.size()) {
    throw std::invalid_argument("bad number: '" + s + "'");
  }
  return v;
}

void triple(const std::string& v, std::array<double, estimation::kImmModes>& out) {
  const auto parts = split(v);
  if (parts.size() != estimation::kImmModes) {
    throw std::invalid_argument("expected three comma-separated values: " + v);
  }
  for (std::size_t i = 0; i < parts.size(); ++i) {
    out[i] = number(parts[i]);
  }
}

Options parse_args(int argc, char** argv) {
  Options o;
  o.imm.motion_model = estimation::MotionModel::Imm;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "-h" || a == "--help") {
      std::fputs(kUsage, stdout);
      std::exit(0);
    }
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value for " + a);
    }
    const std::string v = argv[++i];
    o.args.push_back(a);
    o.args.push_back(v);
    if (a == "--scenarios") {
      o.paths.push_back(v);
    } else if (a == "--title") {
      o.title = v;
    } else if (a == "--imm-accel-psd") {
      triple(v, o.imm.imm.model.accel_psd);
    } else if (a == "--imm-sojourn") {
      triple(v, o.imm.imm.model.sojourn_s);
    } else if (a == "--imm-probability-floor") {
      o.imm.imm.model.probability_floor = number(v);
    } else if (a == "--out") {
      o.out = v;
    } else {
      throw std::invalid_argument("unknown option: " + a);
    }
  }
  if (o.paths.empty()) {
    throw std::invalid_argument("give at least one --scenarios");
  }
  if (!is_valid(o.imm)) {
    throw std::invalid_argument("invalid IMM parameters");
  }
  return o;
}

std::map<std::string, std::string> discover(const std::vector<std::string>& paths) {
  std::map<std::string, std::string> dirs;  // name -> directory, sorted
  auto add = [&](const fs::path& dir) {
    if (!dirs.emplace(dir.filename().string(), dir.string()).second) {
      throw std::invalid_argument("two scenarios named " + dir.filename().string());
    }
  };
  for (const std::string& p : paths) {
    fs::path path(p);
    while (!path.empty() && !path.has_filename()) {
      path = path.parent_path();
    }
    if (is_scenario_dir(path.string())) {
      add(path);
      continue;
    }
    std::size_t found = 0;
    for (const fs::directory_entry& e : fs::directory_iterator(path)) {
      if (e.is_directory() && is_scenario_dir(e.path().string())) {
        add(e.path());
        ++found;
      }
    }
    if (found == 0) {
      throw std::invalid_argument(p + ": no scenario found");
    }
  }
  return dirs;
}

// "<condition>_s<digits>" -> "<condition>"; anything else is its own condition.
std::string condition_of(const std::string& name) {
  const std::size_t u = name.rfind("_s");
  if (u == std::string::npos || u + 2 >= name.size()) {
    return name;
  }
  for (std::size_t i = u + 2; i < name.size(); ++i) {
    if (name[i] < '0' || name[i] > '9') {
      return name;
    }
  }
  return name.substr(0, u);
}

// Sort key: manoeuvre_<g>g_<v>mps conditions by speed then load, after them
// everything else by name.
std::tuple<int, double, double, std::string> sort_key(const std::string& c) {
  double g = 0.0;
  double v = 0.0;
  const std::size_t gpos = c.find("g_");
  const std::size_t vpos = c.find("mps");
  const std::size_t start = c.rfind('_', gpos == std::string::npos ? 0 : gpos);
  if (c.rfind("manoeuvre_", 0) == 0 && gpos != std::string::npos && vpos != std::string::npos &&
      start != std::string::npos) {
    const std::string gs = c.substr(start + 1, gpos - start - 1);
    const std::string vs = c.substr(gpos + 2, vpos - gpos - 2);
    try {
      g = number(gs);
      v = number(vs);
      return {0, v, g, c};
    } catch (const std::invalid_argument&) {
    }
  }
  return {1, 0.0, 0.0, c};
}

enum Phase { kStraight, kOnset, kTurn, kExit, kPhases };
const char* const kPhaseNames[kPhases] = {"straight", "onset", "turn", "exit"};

// Phase of every (target, truth time), from the truth velocities.
std::map<std::pair<int, std::int64_t>, Phase> phases(const Scenario& sc) {
  std::map<int, std::vector<const TruthRow*>> by_target;
  for (const TruthRow& r : sc.truth) {
    by_target[r.target].push_back(&r);
  }
  std::map<std::pair<int, std::int64_t>, Phase> out;
  for (const auto& [tgt, rows] : by_target) {
    bool turning = false;
    std::int64_t onset = 0;
    std::int64_t exit = 0;
    bool had_turn = false;
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const TruthRow& r = *rows[i];
      bool now = false;
      if (i > 0) {
        const TruthRow& p = *rows[i - 1];
        const double dt = static_cast<double>(r.time_ns - p.time_ns) * 1e-9;
        now = dt > 0.0 && std::hypot(r.vx - p.vx, r.vy - p.vy) > kTurnAccel * dt;
      }
      if (now && !turning) {
        onset = r.time_ns;
      }
      if (!now && turning) {
        exit = r.time_ns;
        had_turn = true;
      }
      turning = now;
      Phase ph = kStraight;
      if (now) {
        ph = static_cast<double>(r.time_ns - onset) * 1e-9 < kPhaseS ? kOnset : kTurn;
      } else if (had_turn && static_cast<double>(r.time_ns - exit) * 1e-9 < kPhaseS) {
        ph = kExit;
      }
      out[{tgt, r.time_ns}] = ph;
    }
  }
  return out;
}

struct Totals {
  std::size_t runs = 0;
  double id_switches = 0.0;
  double missed_pct = 0.0;  // sum of per-run percentages
  double false_tracks = 0.0;
  double rmse = 0.0;
  double vel_rmse = 0.0;
  std::size_t rmse_runs = 0;
};

struct Consistency {
  ChiSquareStat nees[kPhases];
  ChiSquareStat nis[kPhases];
};

struct ModelResults {
  std::map<std::string, Totals> by_condition;
  Consistency manoeuvre;  // pooled over manoeuvre_* conditions
  Consistency fixtures;   // pooled over the rest
};

void run_model(const std::map<std::string, std::string>& dirs, const EngineConfig& cfg,
               ModelResults& out) {
  for (const auto& [name, dir] : dirs) {
    const Scenario sc = load_scenario_dir(dir);
    const Score s = run_engine(sc, cfg);
    const std::string cond = condition_of(name);
    Totals& t = out.by_condition[cond];
    ++t.runs;
    t.id_switches += static_cast<double>(s.id_switches);
    t.missed_pct += 100.0 * s.missed_ratio();
    t.false_tracks += static_cast<double>(s.false_tracks);
    if (!s.rmse.empty()) {
      t.rmse += s.mean_rmse;
      t.vel_rmse += s.mean_velocity_rmse;
      ++t.rmse_runs;
    }
    Consistency& c = cond.rfind("manoeuvre_", 0) == 0 ? out.manoeuvre : out.fixtures;
    const auto ph = phases(sc);
    for (const InstantRecord& rec : s.instants) {
      for (const auto& [tgt, nees] : rec.nees) {
        const auto it = ph.find({tgt, rec.time_ns});
        const Phase p = it == ph.end() ? kStraight : it->second;
        c.nees[p].add(nees, 4);
        if (const auto n = rec.nis.find(tgt); n != rec.nis.end()) {
          c.nis[p].merge(n->second);
        }
      }
    }
  }
}

std::string fixed(double v, int precision) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.*f", precision, v);
  return buf;
}

std::string per_dof(const ChiSquareStat& s) {
  return s.n == 0 ? "-" : fixed(s.sum / static_cast<double>(s.dof), 2);
}

std::string markdown(const Options& o, const std::map<std::string, std::string>& dirs,
                     const ModelResults& cv, const ModelResults& imm) {
  std::string out = "# " + o.title + "\n\n";
  out +=
      "<!-- Generated by sifted_motion_eval (eval/sifted_motion_eval.cpp). Do not edit by hand. "
      "-->\n\n";
  out +=
      "Legacy estimator against the IMM (ADR 0014), side by side on the same inputs. "
      "Reproduce from the repository root (the Monte Carlo sets are regenerated, never "
      "committed):\n\n```bash\ncmake --preset release && cmake --build --preset release\n"
      "(cd sim && python3 -m sifted_sim.manoeuvres evaluation ../build/release/manoeuvres-eval)\n"
      "build/release/eval/sifted_motion_eval";
  for (const std::string& a : o.args) {
    out += a.find(' ') == std::string::npos ? " " + a : " \"" + a + "\"";
  }
  out += "\n```\n\n";
  out +=
      "## Models compared\n\n"
      "| | `cv` (legacy) | `imm` (default since ADR 0014) |\n|---|---|---|\n"
      "| state | `[px, py, vx, vy]` | the same, in each of three modes |\n"
      "| motion | one constant-velocity model, white-noise acceleration q = 4 m^2/s^3, x10 "
      "while a fading-memory NIS detector reports a manoeuvre | quiet / manoeuvre / hard "
      "constant-velocity modes, switched by a continuous-time Markov chain |\n"
      "| update | Joseph-form EKF | Joseph-form EKF per mode, each linearised at its own state; "
      "log-space mode likelihoods |\n"
      "| gate and cost | d^2 under the filter's prediction | the mixture statistic over the "
      "modes (ADR 0014) |\n"
      "| reported estimate | the filter's | the moment-matched combination of the modes |\n\n"
      "Everything else (gate probability, global nearest neighbour, lifecycle, sensors) is the "
      "same. Old recordings replay with `cv`.\n\n"
      "## Method\n\n" +
      std::to_string(dirs.size()) +
      " scenario runs per model. The manoeuvre set (`manoeuvre_<load>g_<speed>mps`, 10 noise "
      "seeds each, `sim/sifted_sim/manoeuvres.py`): one radar at the origin (10 m, 0.2 deg, "
      "1 Hz, Pd 0.9, 0.5 clutter points per scan, 60 km), one target from (-6000, 15000) m "
      "flying east: 30 s straight, a right turn of min(180 deg / rate, 40 s), 30 s straight. "
      "Its seeds and turn profiles are disjoint from the tuning set the parameters were "
      "checked on (ADR 0014).\n\n";
  out += "`cv`: the legacy constant-velocity filter with NIS-switched process noise. ";
  out += "`imm`: the IMM (ADR 0014) with q = ";
  const auto& m = o.imm.imm.model;
  out += fixed(m.accel_psd[0], 2) + " / " + fixed(m.accel_psd[1], 1) + " / " +
         fixed(m.accel_psd[2], 0) + " m^2/s^3, sojourn " + fixed(m.sojourn_s[0], 0) + " / " +
         fixed(m.sojourn_s[1], 0) + " / " + fixed(m.sojourn_s[2], 0) + " s. ";
  out +=
      "Direct engine feed, default configuration otherwise. Per condition, averages over its "
      "runs: ID switches and false tracks per run, missed target-instants (mean of per-run "
      "percentages), mean per-target position and velocity RMSE.\n\n"
      "Metrics follow the scorecard (CLEAR-MOT style, 100 m match distance; "
      "eval/sifted/eval/scenario.hpp). An ID switch is a target matched to a different track "
      "than before, usually because the original track lost the target and a new one was "
      "started. A missed target-instant is a covered target with no matched track. At 15-25 km "
      "one radar measurement's cross-range error is 50-90 m, so part of the remaining misses "
      "are instants where the estimate is further than 100 m from the truth. NEES and NIS test "
      "the reported covariance (below). The fixture rows are single noise realisations; the "
      "Monte Carlo evaluation ([evaluation-monte-carlo.md](evaluation-monte-carlo.md)) scores "
      "them over many.\n\n"
      "## Results\n\n";
  out +=
      "| Condition | Runs | ID sw cv | ID sw imm | Missed cv | Missed imm | False cv | False imm "
      "| RMSE cv [m] | RMSE imm [m] | Vel RMSE cv [m/s] | Vel RMSE imm [m/s] |\n";
  out += "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  std::vector<std::string> conds;
  for (const auto& [c, t] : cv.by_condition) {
    conds.push_back(c);
  }
  std::sort(conds.begin(), conds.end(),
            [](const std::string& a, const std::string& b) { return sort_key(a) < sort_key(b); });
  for (const std::string& c : conds) {
    const Totals& a = cv.by_condition.at(c);
    const Totals& b = imm.by_condition.at(c);
    const double n = static_cast<double>(a.runs);
    auto rm = [](const Totals& t, double v) {
      return t.rmse_runs ? fixed(v / static_cast<double>(t.rmse_runs), 1) : std::string("-");
    };
    out += "| " + c + " | " + std::to_string(a.runs) + " | " + fixed(a.id_switches / n, 1) + " | " +
           fixed(b.id_switches / n, 1) + " | " + fixed(a.missed_pct / n, 1) + "% | " +
           fixed(b.missed_pct / n, 1) + "% | " + fixed(a.false_tracks / n, 1) + " | " +
           fixed(b.false_tracks / n, 1) + " | " + rm(a, a.rmse) + " | " + rm(b, b.rmse) + " | " +
           rm(a, a.vel_rmse) + " | " + rm(b, b.vel_rmse) + " |\n";
  }
  auto consistency = [&](const char* label, const Consistency& a, const Consistency& b) {
    bool any = false;
    for (int p = 0; p < kPhases; ++p) {
      any = any || a.nees[p].n > 0;
    }
    if (!any) {
      return;
    }
    out += "\n**Consistency, " + std::string(label) +
           "** (NEES4 expected 4; NIS per degree of freedom expected 1; samples pooled over runs, "
           "so time-correlated: see eval/sifted/eval/consistency.hpp):\n\n";
    out += "| Phase | NEES4 cv | NEES4 imm | NIS/dof cv | NIS/dof imm | NEES samples |\n";
    out += "|---|---:|---:|---:|---:|---:|\n";
    for (int p = 0; p < kPhases; ++p) {
      out += std::string("| ") + kPhaseNames[p] + " | " +
             (a.nees[p].n ? fixed(a.nees[p].average(), 2) : "-") + " | " +
             (b.nees[p].n ? fixed(b.nees[p].average(), 2) : "-") + " | " + per_dof(a.nis[p]) +
             " | " + per_dof(b.nis[p]) + " | " + std::to_string(a.nees[p].n) + " / " +
             std::to_string(b.nees[p].n) + " |\n";
    }
  };
  consistency("manoeuvre scenarios", cv.manoeuvre, imm.manoeuvre);
  consistency("other scenarios", cv.fixtures, imm.fixtures);
  out += "\nPhases come from the truth: `turn` when the true velocity changes by more than " +
         fixed(kTurnAccel, 1) + " m/s^2, `onset` / `exit` the first " + fixed(kPhaseS, 0) +
         " s of a turn / of the straight flight after it.\n";

  // ---- findings, computed from the tables above ------------------------------
  out += "\n## Findings\n\n";
  // ADR 0014's target: at most about 0.2 ID switches per run through 3 g.
  constexpr double kTargetSwitches = 0.2;
  constexpr double kTargetLoadG = 3.0;
  std::map<double, std::pair<double, double>> worst;  // speed -> (cv, imm) through 3 g
  std::map<double, double> first_miss;                // speed -> lowest load above target (imm)
  std::size_t fixtures = 0;
  std::size_t fixtures_better = 0;
  for (const std::string& c : conds) {
    const auto [kind, v, g, name] = sort_key(c);
    const Totals& a = cv.by_condition.at(c);
    const Totals& b = imm.by_condition.at(c);
    const double n = static_cast<double>(a.runs);
    if (kind == 0) {
      if (g <= kTargetLoadG) {
        auto& w = worst[v];
        w.first = std::max(w.first, a.id_switches / n);
        w.second = std::max(w.second, b.id_switches / n);
      }
      if (b.id_switches / n > kTargetSwitches && first_miss.count(v) == 0) {
        first_miss[v] = g;
      }
    } else if (a.rmse_runs > 0 && b.rmse_runs > 0) {
      ++fixtures;
      fixtures_better +=
          b.rmse / static_cast<double>(b.rmse_runs) < a.rmse / static_cast<double>(a.rmse_runs)
              ? 1U
              : 0U;
    }
  }
  for (const auto& [v, w] : worst) {
    out += "- **Identity through " + fixed(kTargetLoadG, 0) + " g at " + fixed(v, 0) +
           " m/s:** at most " + fixed(w.second, 1) + " ID switches per run with `imm`, " +
           fixed(w.first, 1) + " with `cv`. Target (at most about " + fixed(kTargetSwitches, 1) +
           "): **" + (w.second <= kTargetSwitches ? "met" : "not met") + "**";
    if (const auto it = first_miss.find(v); it != first_miss.end()) {
      out += "; `imm` first exceeds it at " + fixed(it->second, 1) + " g";
    }
    out += ".\n";
  }
  if (fixtures > 0) {
    out += "- **Fixtures:** `imm` has the lower mean position RMSE on " +
           std::to_string(fixtures_better) + " of " + std::to_string(fixtures) +
           " single-realisation fixtures.\n";
  }
  const ChiSquareStat& sc = cv.manoeuvre.nees[kStraight];
  const ChiSquareStat& si = imm.manoeuvre.nees[kStraight];
  const ChiSquareStat& tc = cv.manoeuvre.nees[kTurn];
  const ChiSquareStat& ti = imm.manoeuvre.nees[kTurn];
  if (sc.n && si.n && tc.n && ti.n) {
    out += "- **Consistency:** in turns NEES4 goes from " + fixed(tc.average(), 1) + " to " +
           fixed(ti.average(), 1) + " (expected 4). In straight flight it goes from " +
           fixed(sc.average(), 2) + " to " + fixed(si.average(), 2) +
           ": the IMM does not make straight flight honest. The simulator's straight legs have "
           "exactly zero acceleration, so any filter that budgets for manoeuvres is pessimistic "
           "there, and the mixed-in manoeuvre modes add to that budget (docs/design.md "
           "section 7).\n";
  }
  out +=
      "\n## Costs, measured elsewhere\n\n"
      "- **Compute:** per-scan time, throughput and tracks per core for both estimators on the "
      "same machine: [benchmarks.md](benchmarks.md).\n"
      "- **Clutter:** the IMM confirms more false tracks than a constant-velocity tracker as "
      "clutter rises: [evaluation-baseline.md](evaluation-baseline.md) and "
      "[evaluation-monte-carlo.md](evaluation-monte-carlo.md).\n"
      "- **Dense formations and gating:** identity in a 45 m formation over re-seeded fixtures, "
      "the gating statistics compared on separate selection sets, and the engine-level loss "
      "rate on the IMM's own matched model: [ADR 0014](adr/0014-imm-motion-model.md) and "
      "`ConsistencyMonteCarlo.ImmMatchedModelIsNearlyConsistent`.\n";
  return out;
}

}  // namespace
}  // namespace sifted::eval

int main(int argc, char** argv) {
  using namespace sifted::eval;
  try {
    const Options o = parse_args(argc, argv);
    const auto dirs = discover(o.paths);
    sifted::EngineConfig cv;
    cv.motion_model = sifted::estimation::MotionModel::ConstantVelocity;
    ModelResults rc;
    ModelResults ri;
    run_model(dirs, cv, rc);
    run_model(dirs, o.imm, ri);
    const std::string text = markdown(o, dirs, rc, ri);
    if (o.out.empty()) {
      std::fwrite(text.data(), 1, text.size(), stdout);
      return 0;
    }
    std::ofstream f(o.out, std::ios::binary | std::ios::trunc);
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    if (!f.good()) {
      std::fprintf(stderr, "sifted_motion_eval: cannot write %s\n", o.out.c_str());
      return 1;
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "sifted_motion_eval: %s\n%s", e.what(), kUsage);
    return 2;
  }
}
