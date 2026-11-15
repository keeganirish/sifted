#include "sifted/net/sha1.hpp"

#include <vector>

namespace sifted::net {
namespace {

constexpr std::uint32_t rotl(std::uint32_t x, unsigned n) noexcept {
  return (x << n) | (x >> (32U - n));
}

}  // namespace

std::array<std::uint8_t, 20> sha1(std::span<const std::uint8_t> data) noexcept {
  std::uint32_t h[5] = {0x67452301U, 0xEFCDAB89U, 0x98BADCFEU, 0x10325476U, 0xC3D2E1F0U};
  const std::uint64_t bit_len = static_cast<std::uint64_t>(data.size()) * 8U;
  // Process 64-byte blocks, padding the tail as the standard requires.
  const std::size_t total = ((data.size() + 9 + 63) / 64) * 64;
  std::uint8_t block[64];
  for (std::size_t off = 0; off < total; off += 64) {
    for (std::size_t i = 0; i < 64; ++i) {
      const std::size_t k = off + i;
      if (k < data.size()) {
        block[i] = data[k];
      } else if (k == data.size()) {
        block[i] = 0x80;
      } else if (k >= total - 8) {
        block[i] = static_cast<std::uint8_t>(bit_len >> (8 * (total - 1 - k)));
      } else {
        block[i] = 0;
      }
    }
    std::uint32_t w[80];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<std::uint32_t>(block[4 * i]) << 24) |
             (static_cast<std::uint32_t>(block[4 * i + 1]) << 16) |
             (static_cast<std::uint32_t>(block[4 * i + 2]) << 8) |
             static_cast<std::uint32_t>(block[4 * i + 3]);
    }
    for (int i = 16; i < 80; ++i) {
      w[i] = rotl(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }
    std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
    for (int i = 0; i < 80; ++i) {
      std::uint32_t f = 0;
      std::uint32_t k = 0;
      if (i < 20) {
        f = (b & c) | (~b & d);
        k = 0x5A827999U;
      } else if (i < 40) {
        f = b ^ c ^ d;
        k = 0x6ED9EBA1U;
      } else if (i < 60) {
        f = (b & c) | (b & d) | (c & d);
        k = 0x8F1BBCDCU;
      } else {
        f = b ^ c ^ d;
        k = 0xCA62C1D6U;
      }
      const std::uint32_t t = rotl(a, 5) + f + e + k + w[i];
      e = d;
      d = c;
      c = rotl(b, 30);
      b = a;
      a = t;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
  }
  std::array<std::uint8_t, 20> out{};
  for (std::size_t i = 0; i < 20; ++i) {
    out[i] = static_cast<std::uint8_t>(h[i / 4] >> (24 - 8 * (i % 4)));
  }
  return out;
}

std::string base64_encode(std::span<const std::uint8_t> data) {
  static constexpr char kAlphabet[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((data.size() + 2) / 3 * 4);
  for (std::size_t i = 0; i < data.size(); i += 3) {
    const std::uint32_t n =
        (static_cast<std::uint32_t>(data[i]) << 16) |
        (i + 1 < data.size() ? static_cast<std::uint32_t>(data[i + 1]) << 8 : 0U) |
        (i + 2 < data.size() ? static_cast<std::uint32_t>(data[i + 2]) : 0U);
    out += kAlphabet[(n >> 18) & 63U];
    out += kAlphabet[(n >> 12) & 63U];
    out += i + 1 < data.size() ? kAlphabet[(n >> 6) & 63U] : '=';
    out += i + 2 < data.size() ? kAlphabet[n & 63U] : '=';
  }
  return out;
}

}  // namespace sifted::net
