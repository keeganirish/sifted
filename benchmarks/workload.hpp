#pragma once

// Seeded synthetic workloads for the benchmarks (ADR 0011). One radar at the
// origin scans a disc of radius `area_m` once per `period_s`; `targets`
// objects fly straight at 50-250 m/s from uniformly random positions and
// headings (re-entering on the far side when they leave the disc, so the
// target count is constant); each is detected with probability `p_detect`
// with the simulator's noise (10 m, 0.2 deg); `clutter` false alarms per scan
// are uniform over the disc. Every scan is one group (one sensor, one time),
// as the ingest pipeline delivers a radar scan. Same seed, same workload,
// on every machine (SplitMix64 + Box-Muller; libm-dependent only in the last
// bits of the noise, which do not matter for timing).

#include <cmath>
#include <cstdint>
#include <vector>

#include "sifted/core/measurement.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/net/impairment.hpp"

namespace sifted::bench {

struct WorkloadConfig {
  std::size_t targets = 100;
  std::size_t clutter = 0;  // false alarms per scan
  double area_m = 50'000.0;
  double period_s = 1.0;
  double p_detect = 0.95;
  std::uint64_t seed = 1;
};

inline SensorTable radar_table() {
  SensorTable t;
  (void)t.add({1, SensorKind::RangeBearing, {0.0, 0.0, 0.0}});
  return t;
}

class Workload {
 public:
  explicit Workload(const WorkloadConfig& c) : c_(c), rng_(c.seed) {
    for (std::size_t i = 0; i < c_.targets; ++i) {
      const double r = c_.area_m * std::sqrt(0.02 + 0.98 * rng_.uniform());
      const double a = 2.0 * kPi * rng_.uniform();
      const double h = 2.0 * kPi * rng_.uniform();
      const double v = 50.0 + 200.0 * rng_.uniform();
      targets_.push_back({r * std::cos(a), r * std::sin(a), v * std::cos(h), v * std::sin(h)});
    }
  }

  // The next scan: detections and clutter at one timestamp, in bearing order.
  std::vector<Measurement> next_scan() {
    const std::int64_t t =
        start_ns_ + static_cast<std::int64_t>(scan_) * static_cast<std::int64_t>(c_.period_s * 1e9);
    std::vector<std::pair<double, double>> rb;  // (bearing, range)
    for (Target& x : targets_) {
      if (scan_ > 0) {
        x.px += x.vx * c_.period_s;
        x.py += x.vy * c_.period_s;
        if (std::hypot(x.px, x.py) > c_.area_m) {  // re-enter on the other side
          x.px = -x.px * 0.98;
          x.py = -x.py * 0.98;
        }
      }
      const double nr = normal();
      const double nb = normal();
      if (rng_.uniform() < c_.p_detect) {
        rb.emplace_back(std::remainder(std::atan2(x.py, x.px) + kSigmaB * nb, 2.0 * kPi),
                        std::max(1.0, std::hypot(x.px, x.py) + kSigmaR * nr));
      }
    }
    for (std::size_t k = 0; k < c_.clutter; ++k) {
      rb.emplace_back(2.0 * kPi * rng_.uniform() - kPi, c_.area_m * std::sqrt(rng_.uniform()));
    }
    std::sort(rb.begin(), rb.end());
    std::vector<Measurement> scan;
    scan.reserve(rb.size());
    for (const auto& [b, r] : rb) {
      Measurement m;
      m.sensor_id = 1;
      m.kind = SensorKind::RangeBearing;
      m.seq = seq_++;
      m.time = Timestamp{t};
      m.z = {r, b, 0.0};
      m.sigma = {kSigmaR, kSigmaB, 0.0};
      scan.push_back(m);
    }
    ++scan_;
    return scan;
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kSigmaR = 10.0;
  static constexpr double kSigmaB = 0.2 * kPi / 180.0;
  struct Target {
    double px, py, vx, vy;
  };
  double normal() {
    if (spare_) {
      spare_ = false;
      return spare_value_;
    }
    const double u1 = 1.0 - rng_.uniform();
    const double u2 = rng_.uniform();
    const double mag = std::sqrt(-2.0 * std::log(u1));
    spare_value_ = mag * std::sin(2.0 * kPi * u2);
    spare_ = true;
    return mag * std::cos(2.0 * kPi * u2);
  }

  WorkloadConfig c_;
  net::SplitMix64 rng_;
  std::vector<Target> targets_;
  std::size_t scan_ = 0;
  std::uint32_t seq_ = 0;
  std::int64_t start_ns_ = 1'700'000'000'000'000'000;
  bool spare_ = false;
  double spare_value_ = 0.0;
};

}  // namespace sifted::bench
