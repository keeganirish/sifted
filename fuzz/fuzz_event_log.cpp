// libFuzzer target for the event log reader (threat model B4: a log file is
// semi-trusted input).
//
// Properties checked on every input:
//   1. reading never crashes, hangs, over-allocates, or reads out of bounds
//      (ASan/UBSan; the header size cap is checked before allocating);
//   2. any log that opens has a header whose canonical form re-parses to the
//      same RunHeader;
//   3. replaying whatever records it holds through the real pipeline (wire
//      decoder + engine) never crashes.
//
// Run:  cmake --preset fuzz && cmake --build --preset fuzz --target fuzz_event_log
//       mkdir -p build/fuzz-work-log && build/fuzz/fuzz/fuzz_event_log build/fuzz-work-log
//       fuzz/corpus_log -max_total_time=60

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <sstream>
#include <string>

#include "sifted/net/ingest.hpp"
#include "sifted/replay/event_log.hpp"
#include "sifted/replay/replayer.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  std::istringstream in(std::string(reinterpret_cast<const char*>(data), size));  // NOLINT
  auto reader = sifted::replay::EventLogReader::open(in);
  if (!reader.ok()) {
    return 0;
  }
  const auto& h = reader.value().header();
  const auto again = sifted::replay::parse_header(sifted::replay::format_header(h));
  if (!again.ok() || !(again.value() == h)) {
    std::abort();  // header round trip violated
  }
  sifted::net::PipelineConfig cfg = h.config;
  cfg.engine.max_tracks = std::min<std::size_t>(cfg.engine.max_tracks, 64);  // keep runs fast
  sifted::net::Pipeline pipeline(h.sensors, cfg);
  (void)sifted::replay::replay(reader.value(), pipeline);
  return 0;
}
