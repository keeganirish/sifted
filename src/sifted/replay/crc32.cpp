#include "sifted/replay/crc32.hpp"

#include <array>

namespace sifted::replay {
namespace {

constexpr std::array<std::uint32_t, 256> make_table() {
  std::array<std::uint32_t, 256> t{};
  for (std::uint32_t i = 0; i < 256; ++i) {
    std::uint32_t c = i;
    for (int k = 0; k < 8; ++k) {
      c = (c & 1U) != 0 ? 0xEDB88320U ^ (c >> 1) : c >> 1;
    }
    t[i] = c;
  }
  return t;
}
constexpr std::array<std::uint32_t, 256> kTable = make_table();

}  // namespace

std::uint32_t crc32(std::span<const std::uint8_t> bytes, std::uint32_t crc) noexcept {
  crc = ~crc;
  for (std::uint8_t b : bytes) {
    crc = kTable[(crc ^ b) & 0xFFU] ^ (crc >> 8);
  }
  return ~crc;
}

}  // namespace sifted::replay
