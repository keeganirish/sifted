// sifted_eval smoke test: the scorecard tool runs on a small matrix (two
// fixtures, the clean level, one seed), writes byte-identical output twice,
// and its numbers are the tests' numbers:
//   * its direct-feed rows equal, digit for digit, what run_engine (the
//     MultiTargetRegression path) computes, and the baselines recorded in
//     multi_target_regression_test.cpp and docs/design.md section 7;
//   * its clean pipeline row equals run_pipeline over the clean level (the
//     DegradedNetwork path).
// Scenario names appear here, never in the tool.

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "scenario_support.hpp"
#include "sifted/net/impairment.hpp"

namespace sifted {
namespace {

#ifdef SIFTED_EVAL_BIN

using Row = std::map<std::string, std::string>;

std::string slurp(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

std::vector<std::string> fields(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  std::istringstream ls(line);
  while (std::getline(ls, cur, ',')) {
    out.push_back(cur);
  }
  if (!line.empty() && line.back() == ',') {
    out.emplace_back();
  }
  return out;
}

std::vector<Row> parse_csv(const std::string& text) {
  std::istringstream in(text);
  std::string line;
  std::getline(in, line);
  const std::vector<std::string> header = fields(line);
  std::vector<Row> rows;
  while (std::getline(in, line)) {
    const std::vector<std::string> f = fields(line);
    EXPECT_EQ(f.size(), header.size()) << line;
    Row r;
    for (std::size_t i = 0; i < header.size() && i < f.size(); ++i) {
      r[header[i]] = f[i];
    }
    rows.push_back(std::move(r));
  }
  return rows;
}

const Row* find(const std::vector<Row>& rows, const std::string& scenario,
                const std::string& condition, const std::string& window) {
  for (const Row& r : rows) {
    if (r.at("scenario") == scenario && r.at("condition") == condition &&
        r.at("window_s") == window) {
      return &r;
    }
  }
  return nullptr;
}

std::string f2(double v) {
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.2f", v);
  return buf;
}

std::string per_target(const testing::Score& s) {
  std::string out;
  for (const auto& [tgt, r] : s.rmse) {
    out += (out.empty() ? "" : " ") + std::to_string(tgt) + ":" + f2(r);
  }
  return out;
}

void expect_score(const Row& row, const testing::Score& s, const std::string& what) {
  EXPECT_EQ(row.at("mean_rmse_m"), f2(s.mean_rmse)) << what;
  EXPECT_EQ(row.at("worst_rmse_m"), f2(s.max_rmse)) << what;
  EXPECT_EQ(row.at("per_target_rmse_m"), per_target(s)) << what;
  EXPECT_EQ(row.at("missed"), std::to_string(s.missed)) << what;
  EXPECT_EQ(row.at("eligible"), std::to_string(s.eligible)) << what;
  EXPECT_EQ(row.at("id_switches"), std::to_string(s.id_switches)) << what;
  EXPECT_EQ(row.at("false_tracks"), std::to_string(s.false_tracks)) << what;
  EXPECT_EQ(row.at("measurements"), std::to_string(s.measurements)) << what;
  EXPECT_EQ(row.at("out_of_order"), std::to_string(s.out_of_order)) << what;
  // Empty when there is no sample (through the pipeline, the scan stamped T
  // is never applied by T, so no estimate is at its own update instant).
  const auto& u = s.nees.updated.full;
  EXPECT_EQ(row.at("nees4_updated"), u.n ? f2(u.average()) : "") << what;
  const auto& x = s.nees.extrapolated.full;
  EXPECT_EQ(row.at("nees4_extrapolated"), x.n ? f2(x.average()) : "") << what;
}

// The baselines recorded with MultiTargetRegression (direct feed, default
// configuration): per-target RMSE, mean RMSE, eligible target-instants (0
// missed, 0 ID switches, 0 false tracks) and the radar-only mean RMSE.
struct Baseline {
  const char* scenario;
  const char* per_target;
  const char* mean;
  const char* eligible;
  const char* without_bearing_only_mean;
};
constexpr Baseline kBaselines[] = {
    {"crossing_pair", "1:8.39 2:8.63", "8.51", "172", "11.18"},
    {"close_pass", "1:10.49 2:7.64 3:12.20", "10.11", "258", "13.33"},
};

TEST(Eval, ScorecardMatchesRegressionTests) {
  const std::string dir = std::string(SIFTED_FIXTURE_DIR) + "/scenarios/";
  const std::string out = ::testing::TempDir() + "sifted_eval_smoke.csv";
  std::string cmd = std::string("\"") + SIFTED_EVAL_BIN + "\"";
  for (const Baseline& b : kBaselines) {
    cmd += " --scenarios \"" + dir + b.scenario + "\"";
  }
  cmd += " --levels clean --seeds 7 --format csv --out \"" + out + "\"";
  ASSERT_EQ(std::system(cmd.c_str()), 0) << cmd;
  const std::string first = slurp(out);
  ASSERT_EQ(std::system(cmd.c_str()), 0) << cmd;
  EXPECT_EQ(slurp(out), first) << "two runs must write identical bytes";
  const std::vector<Row> rows = parse_csv(first);

  for (const Baseline& b : kBaselines) {
    const testing::Scenario sc = testing::load_scenario(b.scenario);

    // Direct feed: the regression tests' scorer, the recorded baselines.
    const Row* direct = find(rows, b.scenario, "direct", "");
    ASSERT_NE(direct, nullptr) << b.scenario;
    expect_score(*direct, testing::run_engine(sc), b.scenario);
    EXPECT_EQ(direct->at("per_target_rmse_m"), b.per_target) << b.scenario;
    EXPECT_EQ(direct->at("mean_rmse_m"), b.mean) << b.scenario;
    EXPECT_EQ(direct->at("missed"), "0") << b.scenario;
    EXPECT_EQ(direct->at("eligible"), b.eligible) << b.scenario;
    EXPECT_EQ(direct->at("id_switches"), "0") << b.scenario;
    EXPECT_EQ(direct->at("false_tracks"), "0") << b.scenario;

    const Row* radar_only = find(rows, b.scenario, "direct without bearing-only", "");
    ASSERT_NE(radar_only, nullptr) << b.scenario;
    EXPECT_EQ(radar_only->at("mean_rmse_m"), b.without_bearing_only_mean) << b.scenario;

    // Clean pipeline at the daemon's window: the degraded-network path.
    net::ImpairmentConfig ic;
    ic.seed = 7;
    ic.global = testing::impairment_level("clean");
    net::PipelineConfig pc;
    pc.reorder_window_s = testing::kDaemonReorderWindowS;
    const testing::PipelineRun p =
        testing::run_pipeline(sc, net::impair(testing::sent_frames(sc.measurements), ic), pc);
    const Row* clean = find(rows, b.scenario, "clean", f2(testing::kDaemonReorderWindowS));
    ASSERT_NE(clean, nullptr) << b.scenario;
    expect_score(*clean, p.score, std::string(b.scenario) + " clean");
    EXPECT_EQ(clean->at("runs"), "1") << "a clean link is seed-independent, run once";
  }
}

#endif  // SIFTED_EVAL_BIN

}  // namespace
}  // namespace sifted
