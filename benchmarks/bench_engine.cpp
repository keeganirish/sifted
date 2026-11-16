// sifted_bench: engine latency, throughput and tracks-per-core (ADR 0011).
//
//   sifted_bench scan       [--runs R] [--scans S]   per-scan latency vs live tracks and clutter
//   sifted_bench throughput [--runs R]               measurements/s, one engine thread
//   sifted_bench capacity   [--budget-ms B] [--period-s P] [--runs R]
//                           largest target count whose p99 scan time stays within B
//   sifted_bench all        everything above, CSV on stdout
//   sifted_bench profile    one 1000-target configuration, to run under a profiler
// Options: --no-screen (gating without the M3 gate screen, for before/after
// numbers; results are identical, only time differs), --max-targets N.
//
// Build with the release preset. Timing (std::chrono::steady_clock) lives
// only here, around Engine::on_scan; the engine never reads a clock. Each
// configuration is run R times with the same seeded workload; reported are
// the median over runs of each statistic and the spread (min..max over runs)
// of the median, so run-to-run noise is visible. Output is CSV, one row per
// configuration, consumed by benchmarks/report.py.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/core/engine.hpp"
#include "workload.hpp"

namespace {

using namespace sifted;
using Clock = std::chrono::steady_clock;

struct Stats {
  double median = 0, p95 = 0, p99 = 0, max = 0, mean = 0;
};

Stats stats_of(std::vector<double> v) {
  Stats s;
  if (v.empty()) {
    return s;
  }
  std::sort(v.begin(), v.end());
  auto q = [&](double p) {
    const auto i = static_cast<std::size_t>(p * static_cast<double>(v.size() - 1) + 0.5);
    return v[std::min(i, v.size() - 1)];
  };
  s.median = q(0.5);
  s.p95 = q(0.95);
  s.p99 = q(0.99);
  s.max = v.back();
  double sum = 0;
  for (double x : v) {
    sum += x;
  }
  s.mean = sum / static_cast<double>(v.size());
  return s;
}

double median_of(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v.empty() ? 0.0 : v[v.size() / 2];
}

struct RunResult {
  std::vector<double> scan_ms;  // per timed scan
  std::size_t measurements = 0;
  double total_s = 0;
  std::size_t live_tracks = 0;  // mean over timed scans
};

// Warm-up scans let tracks confirm, so the timed scans see a steady state.
bool g_screen = true;              // --no-screen: the M2 gating path, for before/after
std::size_t g_max_targets = 3000;  // --max-targets: cap the scan sweep

RunResult run_once(const bench::WorkloadConfig& wc, std::size_t warmup, std::size_t scans) {
  bench::Workload w(wc);
  EngineConfig cfg;
  cfg.association.gate_screen = g_screen;
  cfg.max_tracks = std::max<std::size_t>(1024, 4 * (wc.targets + wc.clutter));
  Engine engine(bench::radar_table(), cfg);
  std::vector<MeasurementResult> out;
  for (std::size_t i = 0; i < warmup; ++i) {
    const auto scan = w.next_scan();
    engine.on_scan(scan, out);
    out.clear();
  }
  RunResult r;
  std::size_t live_sum = 0;
  for (std::size_t i = 0; i < scans; ++i) {
    const auto scan = w.next_scan();
    const auto t0 = Clock::now();
    engine.on_scan(scan, out);
    const auto t1 = Clock::now();
    out.clear();
    const double s = std::chrono::duration<double>(t1 - t0).count();
    r.scan_ms.push_back(s * 1e3);
    r.total_s += s;
    r.measurements += scan.size();
    live_sum += engine.store().size();
  }
  r.live_tracks = scans ? live_sum / scans : 0;
  return r;
}

struct Row {
  std::size_t targets, clutter, live, scan_size;
  Stats scan;  // medians over runs
  double median_lo, median_hi;
  double us_per_meas;  // median over runs
  double meas_per_s;   // median over runs
};

Row measure(const bench::WorkloadConfig& wc, std::size_t runs, std::size_t warmup,
            std::size_t scans) {
  std::vector<double> med, p95, p99, mx, mean, us, mps;
  std::size_t live = 0;
  std::size_t size = 0;
  for (std::size_t k = 0; k < runs; ++k) {
    const RunResult r = run_once(wc, warmup, scans);
    const Stats s = stats_of(r.scan_ms);
    med.push_back(s.median);
    p95.push_back(s.p95);
    p99.push_back(s.p99);
    mx.push_back(s.max);
    mean.push_back(s.mean);
    us.push_back(r.total_s * 1e6 / static_cast<double>(r.measurements));
    mps.push_back(static_cast<double>(r.measurements) / r.total_s);
    live = r.live_tracks;
    size = r.measurements / scans;
  }
  Row row{wc.targets, wc.clutter, live, size, {}, 0, 0, median_of(us), median_of(mps)};
  row.scan = {median_of(med), median_of(p95), median_of(p99), median_of(mx), median_of(mean)};
  row.median_lo = *std::min_element(med.begin(), med.end());
  row.median_hi = *std::max_element(med.begin(), med.end());
  return row;
}

void header() {
  std::printf(
      "bench,targets,clutter,live_tracks,scan_size,scan_median_ms,scan_p95_ms,scan_p99_ms,"
      "scan_max_ms,scan_median_min_ms,scan_median_max_ms,us_per_measurement,"
      "measurements_per_s\n");
}

void print(const char* bench, const Row& r) {
  std::printf("%s,%zu,%zu,%zu,%zu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.0f\n", bench, r.targets,
              r.clutter, r.live, r.scan_size, r.scan.median, r.scan.p95, r.scan.p99, r.scan.max,
              r.median_lo, r.median_hi, r.us_per_meas, r.meas_per_s);
  std::fflush(stdout);
}

void bench_scan(std::size_t runs, std::size_t scans) {
  for (const std::size_t n : {10UL, 30UL, 100UL, 300UL, 1000UL, 3000UL}) {
    if (n > g_max_targets) {
      break;
    }
    for (const double clutter_ratio : {0.0, 0.1, 1.0}) {
      bench::WorkloadConfig wc;
      wc.targets = n;
      wc.clutter = static_cast<std::size_t>(clutter_ratio * static_cast<double>(n));
      print("scan", measure(wc, runs, 10, n >= 3000 ? scans / 2 : scans));
    }
  }
}

void bench_throughput(std::size_t runs) {
  // Sustained rate of one engine thread on back-to-back scans of a busy
  // picture (300 tracks, 10% clutter): no waiting between scans.
  bench::WorkloadConfig wc;
  wc.targets = 300;
  wc.clutter = 30;
  print("throughput", measure(wc, runs, 10, 200));
}

void bench_capacity(double budget_ms, double period_s, std::size_t runs) {
  // Largest target count N (one scan of all of them per period, 10%
  // clutter) for which one engine thread's p99 scan time stays within
  // budget_ms. The budget must be below the period for the thread to keep
  // up at all; stated with the result. Bisection on N.
  auto p99_at = [&](std::size_t n) {
    bench::WorkloadConfig wc;
    wc.targets = n;
    wc.clutter = n / 10;
    wc.period_s = period_s;
    const Row r = measure(wc, runs, 10, 40);
    print("capacity_probe", r);
    return r.scan.p99;
  };
  std::size_t lo = 10;
  std::size_t hi = 20000;
  if (p99_at(lo) > budget_ms) {
    std::fprintf(stderr, "capacity: even %zu targets exceed the budget\n", lo);
    return;
  }
  while (hi - lo > std::max<std::size_t>(lo / 20, 10)) {  // ~5% resolution
    const std::size_t mid = (lo + hi) / 2;
    (p99_at(mid) <= budget_ms ? lo : hi) = mid;
  }
  std::fprintf(stderr,
               "capacity: %zu tracks per engine thread with p99 scan time <= %.1f ms at %.2f s"
               " update period (10%% clutter)\n",
               lo, budget_ms, period_s);
  std::printf("capacity_result,%zu,%zu,0,0,%.1f,0,0,0,0,0,0,0\n", lo, lo / 10, budget_ms);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string_view mode = argc > 1 ? argv[1] : "all";
  std::size_t runs = 5;
  std::size_t scans = 60;
  double budget_ms = 100.0;
  double period_s = 1.0;
  for (int i = 2; i < argc; i += 2) {
    const std::string_view k = argv[i];
    if (k == "--no-screen") {
      g_screen = false;
      --i;
      continue;
    }
    if (i + 1 >= argc) {
      std::fprintf(stderr, "missing value for %s\n", argv[i]);
      return 2;
    }
    const char* v = argv[i + 1];
    if (k == "--max-targets") {
      g_max_targets = std::strtoul(v, nullptr, 10);
    } else if (k == "--runs") {
      runs = std::strtoul(v, nullptr, 10);
    } else if (k == "--scans") {
      scans = std::strtoul(v, nullptr, 10);
    } else if (k == "--budget-ms") {
      budget_ms = std::strtod(v, nullptr);
    } else if (k == "--period-s") {
      period_s = std::strtod(v, nullptr);
    } else {
      std::fprintf(stderr, "unknown option %s\n", argv[i]);
      return 2;
    }
  }
  if (runs == 0 || scans == 0) {
    return 2;
  }
  header();
  if (mode == "scan" || mode == "all") {
    bench_scan(runs, scans);
  }
  if (mode == "throughput" || mode == "all") {
    bench_throughput(runs);
  }
  if (mode == "profile") {  // one mid-size configuration, for perf/callgrind
    bench::WorkloadConfig wc;
    wc.targets = 1000;
    wc.clutter = 100;
    print("profile", measure(wc, 1, 5, scans));
  }
  if (mode == "capacity" || mode == "all") {
    bench_capacity(budget_ms, period_s, runs);
  }
  return 0;
}
