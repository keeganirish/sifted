// libFuzzer target for the sensor table text parser (threat model B5: trusted
// operator configuration, but still parsed strictly with size and count
// limits, and also embedded in every event log header, B4).
//
// Properties checked on every input:
//   1. parsing never crashes, hangs or over-allocates (ASan/UBSan);
//   2. anything accepted formats to canonical text that parses back to the
//      same table, and formatting that again is a fixed point.
//
// Run:  mkdir -p build/fuzz-work-sensors && build/fuzz/fuzz/fuzz_sensor_table
//       build/fuzz-work-sensors fuzz/corpus_sensors -max_total_time=60

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>

#include "sifted/core/sensor.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  const std::string_view text(reinterpret_cast<const char*>(data), size);  // NOLINT
  const auto table = sifted::parse_sensor_table(text);
  if (!table.ok()) {
    return 0;
  }
  const std::string canonical = sifted::format_sensor_table(table.value());
  const auto again = sifted::parse_sensor_table(canonical);
  if (!again.ok() || !(again.value() == table.value()) ||
      sifted::format_sensor_table(again.value()) != canonical) {
    std::abort();  // canonical round trip violated
  }
  return 0;
}
