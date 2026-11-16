// libFuzzer target for the event log header text parser (threat model B4),
// directly, without the CRC that makes fuzz_event_log reach it rarely.
//
// Properties checked on every input:
//   1. parsing never crashes, hangs or over-allocates (ASan/UBSan);
//   2. any accepted header is valid (net::is_valid holds, so a Pipeline can
//      be built from it) and its canonical text parses back to the same
//      RunHeader, and formatting that is a fixed point.
//
// Run:  mkdir -p build/fuzz-work-header && build/fuzz/fuzz/fuzz_log_header
//       build/fuzz-work-header fuzz/corpus_header -max_total_time=60

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>

#include "sifted/net/ingest.hpp"
#include "sifted/replay/event_log.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  const std::string_view text(reinterpret_cast<const char*>(data), size);  // NOLINT
  const auto h = sifted::replay::parse_header(text);
  if (!h.ok()) {
    return 0;
  }
  if (!sifted::net::is_valid(h.value().config)) {
    std::abort();
  }
  const std::string canonical = sifted::replay::format_header(h.value());
  const auto again = sifted::replay::parse_header(canonical);
  if (!again.ok() || !(again.value() == h.value()) ||
      sifted::replay::format_header(again.value()) != canonical) {
    std::abort();  // canonical round trip violated
  }
  return 0;
}
