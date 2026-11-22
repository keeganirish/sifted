// sifted_montecarlo end to end on a tiny manifest built from committed
// fixtures (no Python needed): it runs, writes byte-identical output twice,
// and Sifted's own tracks exported as track reports and scored back as an
// "external" tracker give the same per-run numbers as in-process scoring.

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace sifted {
namespace {

#ifdef SIFTED_MONTECARLO_BIN

namespace fs = std::filesystem;

std::string slurp(const fs::path& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

int run(const std::string& args) {
  const std::string cmd = std::string("\"") + SIFTED_MONTECARLO_BIN + "\" " + args;
  return std::system(cmd.c_str());
}

fs::path make_manifest() {
  const fs::path root = fs::path(::testing::TempDir()) / "sifted_mc_smoke";
  fs::remove_all(root);
  const fs::path fixtures = fs::path(SIFTED_FIXTURE_DIR) / "scenarios";
  std::string m =
      "sifted-montecarlo 1\nbase_seed,1\ngenerator,test\n"
      "set,condition,scenario,run,seed,impairment_seed,p_detect,clutter_per_scan,dir\n";
  int k = 0;
  for (const char* name : {"close_pass", "formation_split"}) {
    const std::string dir = std::string("noise/default/") + name + "/r000";
    fs::create_directories(root / dir);
    for (const char* f : {"sensors.cfg", "frames.hex", "truth.csv"}) {
      fs::copy_file(fixtures / name / f, root / dir / f);
    }
    m += std::string("noise,default,") + name + ",0," + std::to_string(11 + k) + "," +
         std::to_string(7 + k) + ",,," + dir + "\n";
    ++k;
  }
  // The same fixture as a sweep condition row.
  const std::string sweep = "sweep/clutter0.5_pd0.9/close_pass/r000";
  fs::create_directories(root / sweep);
  for (const char* f : {"sensors.cfg", "frames.hex", "truth.csv"}) {
    fs::copy_file(fixtures / "close_pass" / f, root / sweep / f);
  }
  m += "sweep,clutter0.5_pd0.9,close_pass,0,11,7,0.9,0.5," + sweep + "\n";
  std::ofstream(root / "manifest.csv", std::ios::binary) << m;
  return root;
}

std::map<std::string, std::string> rows_by_key(const std::string& csv, const std::string& tracker) {
  std::map<std::string, std::string> out;
  std::istringstream in(csv);
  std::string line;
  std::getline(in, line);
  while (std::getline(in, line)) {
    const std::size_t c = line.find(',');
    if (line.substr(0, c) != tracker) {
      continue;
    }
    // key: set,condition,scenario,run; value: the metrics, without track_limit
    // (an engine counter, unavailable for external trackers).
    std::size_t pos = c + 1;
    for (int i = 0; i < 4; ++i) {
      pos = line.find(',', pos) + 1;
    }
    const std::string key = line.substr(c + 1, pos - c - 2);
    out[key] = line.substr(pos, line.rfind(',') - pos);
  }
  return out;
}

TEST(MonteCarloSmoke, RunsDeterministicallyAndScoresExportedTracksIdentically) {
  const fs::path root = make_manifest();
  const std::string manifest = "--manifest \"" + (root / "manifest.csv").string() + "\"";
  const fs::path md = root / "page.md";
  ASSERT_EQ(run(manifest + " --bootstrap 200 --out \"" + md.string() + "\""), 0);
  const std::string first = slurp(md);
  ASSERT_EQ(run(manifest + " --bootstrap 200 --out \"" + md.string() + "\""), 0);
  EXPECT_EQ(slurp(md), first) << "two runs must write identical bytes";
  EXPECT_NE(first.find("## Summary per condition"), std::string::npos);
  EXPECT_NE(first.find("network"), std::string::npos) << "the pipeline condition ran";
  EXPECT_EQ(first.find(root.string()), std::string::npos) << "no machine paths on the page";

  // Export Sifted's tracks, then score them back as an external tracker.
  const fs::path tracks = root / "exported";
  ASSERT_EQ(run(manifest + " --network none --format csv --export-tracks \"" + tracks.string() +
                "\" --out \"" + (root / "a.csv").string() + "\""),
            0);
  ASSERT_EQ(run(manifest + " --network none --format csv --tracks again=\"" + tracks.string() +
                "\" --out \"" + (root / "b.csv").string() + "\""),
            0);
  const std::string b = slurp(root / "b.csv");
  const auto in_process = rows_by_key(b, "sifted");
  const auto from_file = rows_by_key(b, "again");
  ASSERT_EQ(in_process.size(), 3U);
  EXPECT_EQ(in_process, from_file);
  EXPECT_EQ(slurp(root / "a.csv"), [&] {  // the export run scored the same
    std::string only;
    std::istringstream in(b);
    std::string line;
    while (std::getline(in, line)) {
      if (line.rfind("again,", 0) != 0) {
        only += line + "\n";
      }
    }
    return only;
  }());

  // Malformed input is refused with a clear error, not scored.
  std::ofstream(tracks / "noise/default/close_pass/r000.tracks", std::ios::binary)
      << "sifted-tracks 1\nbad\n";
  EXPECT_NE(run(manifest + " --network none --tracks again=\"" + tracks.string() + "\" --out \"" +
                (root / "c.md").string() + "\" 2>" + (root / "err.txt").string()),
            0);
  EXPECT_NE(slurp(root / "err.txt").find("track report line 2"), std::string::npos);
}

#endif  // SIFTED_MONTECARLO_BIN

}  // namespace
}  // namespace sifted
