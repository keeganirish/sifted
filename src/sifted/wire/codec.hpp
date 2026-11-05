#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include "sifted/core/measurement.hpp"
#include "sifted/core/result.hpp"

// Sifted wire format, version 1. Normative description: docs/adr/0001-wire-format.md
//
// One UDP datagram carries exactly one frame:
//
//   offset size  field
//   0      4     magic "SIFT"
//   4      1     version (1)
//   5      1     message type (1 = Measurement)
//   6      2     payload length, little-endian
//   8      n     payload
//
// All integers are little-endian; doubles are IEEE-754 binary64 bit patterns.
// Decoding is strict: anything not produced by encode() is rejected, so a
// frame has exactly one valid byte representation (decode(encode(m)) == m and
// encode(decode(b)) == b). The fuzz target checks that property.
namespace sifted::wire {

inline constexpr std::array<std::uint8_t, 4> kMagic{'S', 'I', 'F', 'T'};
inline constexpr std::uint8_t kVersion = 1;
inline constexpr std::size_t kHeaderSize = 8;
// Hard cap on payload size, enforced before any payload is interpreted.
inline constexpr std::size_t kMaxPayloadSize = 1024;

enum class MessageType : std::uint8_t { Measurement = 1 };

// sensor_id(2) kind(1) reserved(1) seq(4) time_ns(8) z(3*8) sigma(3*8)
inline constexpr std::size_t kMeasurementPayloadSize = 64;
inline constexpr std::size_t kMeasurementFrameSize = kHeaderSize + kMeasurementPayloadSize;

enum class DecodeError : std::uint8_t {
  Truncated,  // shorter than the header or than the length it declares
  BadMagic,
  UnsupportedVersion,
  UnknownMessageType,
  PayloadTooLarge,  // declared length exceeds kMaxPayloadSize
  LengthMismatch,   // trailing bytes, or wrong payload size for the type
  NonZeroReserved,
  UnknownSensorKind,
  InvalidMeasurement,  // failed sifted::is_valid()
};

std::string_view to_string(DecodeError e) noexcept;

// Serializes `m` as one complete frame (kMeasurementFrameSize bytes). Does not
// validate `m`; decode_measurement() will reject an invalid one on the way in.
std::vector<std::uint8_t> encode(const Measurement& m);

// Parses one datagram. Never reads outside `datagram`, never allocates, and
// returns a validated Measurement or the reason it was rejected.
Result<Measurement, DecodeError> decode_measurement(
    std::span<const std::uint8_t> datagram) noexcept;

}  // namespace sifted::wire
