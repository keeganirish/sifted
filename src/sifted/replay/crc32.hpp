#pragma once

#include <cstdint>
#include <span>

namespace sifted::replay {

// CRC-32 (IEEE 802.3, reflected, polynomial 0xEDB88320), as used by zlib/PNG.
// crc32("123456789") == 0xCBF43926.
std::uint32_t crc32(std::span<const std::uint8_t> bytes, std::uint32_t crc = 0) noexcept;

}  // namespace sifted::replay
