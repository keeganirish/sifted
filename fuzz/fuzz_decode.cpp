// libFuzzer target for the wire decoder (the untrusted-input boundary).
//
// Properties checked on every input:
//   1. decode never crashes, hangs, or reads out of bounds (via ASan/UBSan);
//   2. anything accepted re-encodes to exactly the input bytes.
//
// Run:  cmake --preset fuzz && cmake --build --preset fuzz
//       mkdir -p build/fuzz-work && build/fuzz/fuzz/fuzz_decode build/fuzz-work fuzz/corpus
//       -max_total_time=60

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <span>
#include <vector>

#include "sifted/wire/codec.hpp"

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  const std::span<const std::uint8_t> input(data, size);
  const auto r = sifted::wire::decode_measurement(input);
  if (r.ok()) {
    const std::vector<std::uint8_t> again = sifted::wire::encode(r.value());
    if (again.size() != size || !std::equal(again.begin(), again.end(), data)) {
      std::abort();  // canonical round-trip violated
    }
  }
  return 0;
}
