#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

// SHA-1 and base64, only for the WebSocket opening handshake (RFC 6455
// section 4.2.2), where SHA-1 is a fixed part of the protocol, not a security
// mechanism. Do not use for anything else.
namespace sifted::net {

std::array<std::uint8_t, 20> sha1(std::span<const std::uint8_t> data) noexcept;
std::string base64_encode(std::span<const std::uint8_t> data);

}  // namespace sifted::net
