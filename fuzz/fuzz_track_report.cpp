// libFuzzer target for the track report parser (threat model B7: tracker
// output files read by the evaluation, e.g. an external baseline's; untrusted
// developer-tooling input, eval/sifted/eval/track_report.hpp).
//
// Properties checked on every input:
//   1. parsing never crashes, hangs or over-allocates (ASan/UBSan); anything
//      rejected is rejected with TrackReportError, never another exception;
//   2. anything accepted formats to canonical text that parses back to the
//      same report, and formatting that again is a fixed point;
//   3. scoring an accepted report against a small fixed truth either succeeds
//      or fails with TrackReportError (times off the truth instants, reused
//      ids), and never yields a non-finite GOSPA.
//
// Run:  mkdir -p build/fuzz-work-tracks && build/fuzz/fuzz/fuzz_track_report
//       build/fuzz-work-tracks fuzz/corpus_tracks -max_total_time=60

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/eval/track_report.hpp"

namespace {

const std::vector<sifted::eval::TruthRow>& truth() {
  static const std::vector<sifted::eval::TruthRow> t = [] {
    std::vector<sifted::eval::TruthRow> rows;
    for (std::int64_t s = 0; s <= 10; ++s) {
      rows.push_back({s * 1'000'000'000, 100.0 * static_cast<double>(s), 0.0, 1, true, 100.0, 0.0});
      rows.push_back({s * 1'000'000'000, 0.0, 50.0, 2, s > 2, 0.0, 0.0});
    }
    return rows;
  }();
  return t;
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  const std::string_view text(reinterpret_cast<const char*>(data), size);  // NOLINT
  sifted::eval::TrackReport report;
  try {
    report = sifted::eval::parse_track_report(text);
  } catch (const sifted::eval::TrackReportError&) {
    return 0;
  }
  const std::string canonical = sifted::eval::format_track_report(report);
  const sifted::eval::TrackReport again = sifted::eval::parse_track_report(canonical);
  if (!(again == report) || sifted::eval::format_track_report(again) != canonical) {
    std::abort();  // canonical round trip violated
  }
  try {
    const sifted::eval::Score s = sifted::eval::score_track_report(truth(), report);
    if (!std::isfinite(s.gospa.mean_total(s.gospa_params.p))) {
      std::abort();
    }
  } catch (const sifted::eval::TrackReportError&) {
  }
  return 0;
}
