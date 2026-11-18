// libFuzzer target for the event log reader (threat model B4: a log file is
// semi-trusted input).
//
// Properties checked on every input:
//   1. reading never crashes, hangs, over-allocates, or reads out of bounds
//      (ASan/UBSan; the header size cap is checked before allocating);
//   2. any log that opens has a header whose canonical form re-parses to the
//      same RunHeader;
//   3. replaying whatever records it holds through the real pipeline (wire
//      decoder + engine) never crashes, under either tail policy;
//   4. recovery never invents data: the records a TailPolicy::RecoverTorn reader
//      returns start with exactly the records the strict reader returns, it
//      fails whenever the strict reader fails for any reason but a torn tail,
//      and on a log the strict reader accepts it reports nothing truncated.
//
// Run:  cmake --preset fuzz && cmake --build --preset fuzz --target fuzz_event_log
//       mkdir -p build/fuzz-work-log && build/fuzz/fuzz/fuzz_event_log build/fuzz-work-log
//       fuzz/corpus_log -max_total_time=60

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include "sifted/net/ingest.hpp"
#include "sifted/replay/event_log.hpp"
#include "sifted/replay/replayer.hpp"

namespace {

using sifted::replay::EventLogReader;
using sifted::replay::LogError;
using sifted::replay::LogRecord;
using sifted::replay::TailPolicy;

struct ReadAll {
  std::vector<LogRecord> records;
  std::optional<LogError> error;
  bool torn = false;
};

ReadAll read_all(const std::string& bytes, TailPolicy policy) {
  ReadAll out;
  std::istringstream in(bytes);
  auto reader = EventLogReader::open(in, policy);
  if (!reader.ok()) {
    out.error = reader.error();
    return out;
  }
  while (true) {
    auto next = reader.value().next();
    if (!next.ok()) {
      out.error = next.error();
      return out;
    }
    if (!next.value()) {
      out.torn = reader.value().torn_tail();
      return out;
    }
    out.records.push_back(std::move(*next.value()));
  }
}

bool same(const LogRecord& a, const LogRecord& b) {
  return a.type == b.type && a.recv == b.recv && a.payload == b.payload;
}

void replay_with(const std::string& bytes, TailPolicy policy) {
  std::istringstream in(bytes);
  auto reader = EventLogReader::open(in, policy);
  if (!reader.ok()) {
    return;
  }
  sifted::net::PipelineConfig cfg = reader.value().header().config;
  cfg.engine.max_tracks = std::min<std::size_t>(cfg.engine.max_tracks, 64);  // keep runs fast
  sifted::net::Pipeline pipeline(reader.value().header().sensors, cfg);
  (void)sifted::replay::replay(reader.value(), pipeline);
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  const std::string bytes(reinterpret_cast<const char*>(data), size);  // NOLINT
  std::istringstream in(bytes);
  auto reader = EventLogReader::open(in);
  if (!reader.ok()) {
    return 0;
  }
  const auto& h = reader.value().header();
  const auto again = sifted::replay::parse_header(sifted::replay::format_header(h));
  if (!again.ok() || !(again.value() == h)) {
    std::abort();  // header round trip violated
  }

  const ReadAll strict = read_all(bytes, TailPolicy::Strict);
  const ReadAll tolerant = read_all(bytes, TailPolicy::RecoverTorn);
  if (tolerant.records.size() < strict.records.size()) {
    std::abort();  // recovery lost a record the strict reader accepted
  }
  for (std::size_t i = 0; i < strict.records.size(); ++i) {
    if (!same(strict.records[i], tolerant.records[i])) {
      std::abort();
    }
  }
  if (!strict.error &&
      (tolerant.error || tolerant.torn || tolerant.records.size() != strict.records.size())) {
    std::abort();  // a sound log must read the same either way
  }
  const bool tail_error =
      strict.error == LogError::Truncated || strict.error == LogError::BadChecksum;
  if (strict.error && !tail_error && tolerant.error != strict.error) {
    std::abort();  // only a torn tail may be forgiven
  }
  if (tolerant.torn && tolerant.records.size() != strict.records.size()) {
    std::abort();  // what is forgiven is the final record, and nothing after it
  }

  replay_with(bytes, TailPolicy::Strict);
  replay_with(bytes, TailPolicy::RecoverTorn);
  return 0;
}
