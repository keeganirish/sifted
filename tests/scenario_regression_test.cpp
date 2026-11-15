// Ground-truth regression test: simulator output -> C++ decoder -> Engine,
// scored against the simulator's truth.
//
// Fixtures live in tests/fixtures/scenarios/<name>/ and are regenerated with
//   cd sim && python3 -m sifted_sim.generate <name> ../tests/fixtures/scenarios/<name>
// (see tests/fixtures/README.md). Nothing here knows the scenario's trajectory;
// the engine sees only the sensor table and the frames, as it would live.
//
// Thresholds: the reference is the "raw" error, i.e. each measurement converted
// straight to x/y with no filtering. For these radars that error is
// sqrt(sigma_r^2 + (r * sigma_b)^2) ~ 20 m at 5 km. A constant-velocity
// filter with this process noise should average that down by roughly 0.5-0.6x
// on a straight track (steady-state alpha-beta gain for tracking index
// sigma_a * T^2 / sigma_meas ~ 0.1), and lose some of that during a 3 m/s^2
// turn, which the model only absorbs as process noise. Each scenario therefore
// asserts (a) filtered RMSE below a fraction of the raw RMSE and (b) an absolute
// bound, both with margin above the measured value so that seed-level noise
// does not flake but a real regression (wrong Jacobian, broken wrap-around,
// wrong pose transform, bad dt) fails loudly. The first kBurnInSeconds are
// excluded: the track starts from one fix with an unknown velocity.
//
// Baseline when the bounds were set (GCC 13, default EngineConfig, q = 4):
//   straight_crossing  filtered 11.1 m vs raw 22.2 m (0.50), velocity 2.4 m/s
//   turn               filtered 10.7 m vs raw 14.9 m (0.72), velocity 5.3 m/s
// Sanity checks run against these bounds: dropping the bearing wrap in the EKF
// innovation sends straight_crossing to ~10 km RMSE; ignoring sensor yaw sends
// turn to ~1.3 km. Both fail by orders of magnitude, not by a hair.

#include <gtest/gtest.h>

#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/estimation/range_bearing.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted {
namespace {

constexpr double kBurnInSeconds = 10.0;

struct TruthRow {
  std::int64_t time_ns = 0;
  double x = 0.0;
  double y = 0.0;
  double vx = 0.0;
  double vy = 0.0;
};

struct Scenario {
  SensorTable sensors;
  std::vector<std::vector<std::uint8_t>> frames;
  std::vector<TruthRow> truth;
};

std::string read_file(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  EXPECT_TRUE(f.good()) << "cannot open " << path;
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
  EXPECT_TRUE(ec == std::errc{} && ptr == s.data() + s.size()) << "bad number: " << s;
  return v;
}

Scenario load(const std::string& name) {
  const std::string dir = std::string(SIFTED_FIXTURE_DIR) + "/scenarios/" + name + "/";
  Scenario sc;

  const auto table = parse_sensor_table(read_file(dir + "sensors.cfg"));
  EXPECT_TRUE(table.ok());
  if (table.ok()) {
    sc.sensors = table.value();
  }

  std::istringstream frames(read_file(dir + "frames.hex"));
  for (std::string line; frames >> line;) {
    sc.frames.push_back(from_hex(line));
  }

  std::istringstream truth(read_file(dir + "truth.csv"));
  std::string line;
  std::getline(truth, line);  // header
  EXPECT_EQ(line, "time_ns,x_m,y_m,vx_mps,vy_mps");
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
    EXPECT_EQ(f.size(), 5U) << line;
    if (f.size() == 5) {
      sc.truth.push_back({parse<std::int64_t>(f[0]), parse<double>(f[1]), parse<double>(f[2]),
                          parse<double>(f[3]), parse<double>(f[4])});
    }
  }
  return sc;
}

struct Score {
  double filtered_rmse = 0.0;
  double raw_rmse = 0.0;
  double velocity_rmse = 0.0;
  std::size_t scored = 0;
  std::vector<TrackEstimate> estimates;  // one per frame, for determinism checks
};

Score run(const Scenario& sc) {
  Score s;
  Engine engine(sc.sensors);
  EXPECT_EQ(sc.frames.size(), sc.truth.size());
  double sum_f = 0.0;
  double sum_r = 0.0;
  double sum_v = 0.0;
  for (std::size_t i = 0; i < sc.frames.size() && i < sc.truth.size(); ++i) {
    const auto decoded = wire::decode_measurement(sc.frames[i]);
    EXPECT_TRUE(decoded.ok()) << "frame " << i << ": "
                              << (decoded.ok() ? "" : wire::to_string(decoded.error()));
    if (!decoded.ok()) {
      continue;
    }
    const Measurement& m = decoded.value();
    const TruthRow& t = sc.truth[i];
    EXPECT_EQ(m.time.ns, t.time_ns) << "frame " << i;

    const MeasurementOutcome o = engine.on_measurement(m);
    EXPECT_TRUE(o == MeasurementOutcome::Initialized || o == MeasurementOutcome::Updated)
        << "frame " << i << ": " << to_string(o);
    const auto est = engine.track();
    if (!est) {
      continue;
    }
    s.estimates.push_back(*est);

    const double elapsed = static_cast<double>(t.time_ns - sc.truth.front().time_ns) * 1e-9;
    if (elapsed < kBurnInSeconds) {
      continue;
    }
    const SensorConfig* sensor = sc.sensors.find(m.sensor_id);
    const auto raw = estimation::range_bearing_to_local({m.z[0], m.z[1]}, {m.sigma[0], m.sigma[1]},
                                                        sensor->pose);
    sum_f += std::pow(est->state(0) - t.x, 2) + std::pow(est->state(1) - t.y, 2);
    sum_r += std::pow(raw.position(0) - t.x, 2) + std::pow(raw.position(1) - t.y, 2);
    sum_v += std::pow(est->state(2) - t.vx, 2) + std::pow(est->state(3) - t.vy, 2);
    ++s.scored;
  }
  if (s.scored > 0) {
    const auto n = static_cast<double>(s.scored);
    s.filtered_rmse = std::sqrt(sum_f / n);
    s.raw_rmse = std::sqrt(sum_r / n);
    s.velocity_rmse = std::sqrt(sum_v / n);
  }
  return s;
}

struct Bounds {
  double max_ratio_to_raw;  // filtered RMSE / raw-measurement RMSE
  double max_rmse_m;        // absolute 2D position RMSE
  double max_velocity_rmse_mps;
};

void check(const std::string& name, const Bounds& b) {
  const Scenario sc = load(name);
  ASSERT_FALSE(sc.frames.empty());
  const Score s = run(sc);
  ASSERT_GT(s.scored, 50U);
  std::printf(
      "[ scenario ] %s: filtered %.2f m, raw %.2f m (ratio %.3f), velocity %.2f m/s, n=%zu\n",
      name.c_str(), s.filtered_rmse, s.raw_rmse, s.filtered_rmse / s.raw_rmse, s.velocity_rmse,
      s.scored);
  EXPECT_LT(s.filtered_rmse / s.raw_rmse, b.max_ratio_to_raw);
  EXPECT_LT(s.filtered_rmse, b.max_rmse_m);
  EXPECT_LT(s.velocity_rmse, b.max_velocity_rmse_mps);

  // Replaying the same frames gives bit-identical estimates (ADR 0002).
  const Score again = run(sc);
  ASSERT_EQ(again.estimates.size(), s.estimates.size());
  for (std::size_t i = 0; i < s.estimates.size(); ++i) {
    ASSERT_TRUE(again.estimates[i].state == s.estimates[i].state) << "estimate " << i;
    ASSERT_TRUE(again.estimates[i].covariance == s.estimates[i].covariance) << "estimate " << i;
  }
}

}  // namespace

TEST(ScenarioRegression, StraightCrossingBearingSeam) {
  check("straight_crossing", {0.70, 15.0, 3.5});
}

TEST(ScenarioRegression, Turn) { check("turn", {0.85, 14.0, 7.5}); }

}  // namespace sifted
