#include "sifted/wire/codec.hpp"

#include <algorithm>
#include <bit>

namespace sifted::wire {
namespace {

// ---- little-endian writers -------------------------------------------------

void put_u8(std::vector<std::uint8_t>& out, std::uint8_t v) { out.push_back(v); }

void put_u16(std::vector<std::uint8_t>& out, std::uint16_t v) {
  out.push_back(static_cast<std::uint8_t>(v & 0xFFU));
  out.push_back(static_cast<std::uint8_t>((v >> 8) & 0xFFU));
}

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t v) {
  for (unsigned i = 0; i < 4; ++i) {
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
  }
}

void put_u64(std::vector<std::uint8_t>& out, std::uint64_t v) {
  for (unsigned i = 0; i < 8; ++i) {
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
  }
}

void put_f64(std::vector<std::uint8_t>& out, double v) {
  put_u64(out, std::bit_cast<std::uint64_t>(v));
}

// ---- bounds-checked little-endian reader ----------------------------------
// Callers check the total length up front; the reader additionally clamps so a
// logic error there degrades to zeros rather than an out-of-bounds read.

class Reader {
 public:
  explicit Reader(std::span<const std::uint8_t> bytes) noexcept : bytes_(bytes) {}

  std::uint8_t u8() noexcept { return static_cast<std::uint8_t>(uint_le(1)); }
  std::uint16_t u16() noexcept { return static_cast<std::uint16_t>(uint_le(2)); }
  std::uint32_t u32() noexcept { return static_cast<std::uint32_t>(uint_le(4)); }
  std::uint64_t u64() noexcept { return uint_le(8); }
  double f64() noexcept { return std::bit_cast<double>(u64()); }

 private:
  std::uint64_t uint_le(std::size_t n) noexcept {
    std::uint64_t v = 0;
    for (std::size_t i = 0; i < n; ++i) {
      if (pos_ + i < bytes_.size()) {
        v |= static_cast<std::uint64_t>(bytes_[pos_ + i]) << (8 * i);
      }
    }
    pos_ += n;
    return v;
  }

  std::span<const std::uint8_t> bytes_;
  std::size_t pos_ = 0;
};

bool known_kind(std::uint8_t k) noexcept {
  return k >= static_cast<std::uint8_t>(SensorKind::RangeBearing) &&
         k <= static_cast<std::uint8_t>(SensorKind::GeoPosition);
}

}  // namespace

std::string_view to_string(DecodeError e) noexcept {
  switch (e) {
    case DecodeError::Truncated:
      return "truncated";
    case DecodeError::BadMagic:
      return "bad magic";
    case DecodeError::UnsupportedVersion:
      return "unsupported version";
    case DecodeError::UnknownMessageType:
      return "unknown message type";
    case DecodeError::PayloadTooLarge:
      return "payload too large";
    case DecodeError::LengthMismatch:
      return "length mismatch";
    case DecodeError::NonZeroReserved:
      return "non-zero reserved field";
    case DecodeError::UnknownSensorKind:
      return "unknown sensor kind";
    case DecodeError::InvalidMeasurement:
      return "invalid measurement";
  }
  return "unknown error";
}

std::vector<std::uint8_t> encode(const Measurement& m) {
  std::vector<std::uint8_t> out;
  out.reserve(kMeasurementFrameSize);

  out.insert(out.end(), kMagic.begin(), kMagic.end());
  put_u8(out, kVersion);
  put_u8(out, static_cast<std::uint8_t>(MessageType::Measurement));
  put_u16(out, static_cast<std::uint16_t>(kMeasurementPayloadSize));

  put_u16(out, m.sensor_id);
  put_u8(out, static_cast<std::uint8_t>(m.kind));
  put_u8(out, 0);  // reserved
  put_u32(out, m.seq);
  put_u64(out, static_cast<std::uint64_t>(m.time.ns));
  for (double v : m.z) {
    put_f64(out, v);
  }
  for (double v : m.sigma) {
    put_f64(out, v);
  }
  return out;
}

Result<Measurement, DecodeError> decode_measurement(
    std::span<const std::uint8_t> datagram) noexcept {
  // Order matters: each check only relies on what the previous ones proved.
  if (datagram.size() < kHeaderSize) {
    return DecodeError::Truncated;
  }
  if (!std::equal(kMagic.begin(), kMagic.end(), datagram.begin())) {
    return DecodeError::BadMagic;
  }

  Reader header(datagram.subspan(kMagic.size(), kHeaderSize - kMagic.size()));
  const std::uint8_t version = header.u8();
  const std::uint8_t type = header.u8();
  const std::size_t payload_len = header.u16();

  if (version != kVersion) {
    return DecodeError::UnsupportedVersion;
  }
  if (type != static_cast<std::uint8_t>(MessageType::Measurement)) {
    return DecodeError::UnknownMessageType;
  }
  if (payload_len > kMaxPayloadSize) {
    return DecodeError::PayloadTooLarge;
  }
  if (datagram.size() < kHeaderSize + payload_len) {
    return DecodeError::Truncated;
  }
  if (datagram.size() > kHeaderSize + payload_len || payload_len != kMeasurementPayloadSize) {
    return DecodeError::LengthMismatch;
  }

  Reader r(datagram.subspan(kHeaderSize, payload_len));
  Measurement m;
  m.sensor_id = r.u16();
  const std::uint8_t kind = r.u8();
  const std::uint8_t reserved = r.u8();
  m.seq = r.u32();
  m.time.ns = static_cast<std::int64_t>(r.u64());
  for (double& v : m.z) {
    v = r.f64();
  }
  for (double& v : m.sigma) {
    v = r.f64();
  }

  if (reserved != 0) {
    return DecodeError::NonZeroReserved;
  }
  if (!known_kind(kind)) {
    return DecodeError::UnknownSensorKind;
  }
  m.kind = static_cast<SensorKind>(kind);
  if (!is_valid(m)) {
    return DecodeError::InvalidMeasurement;
  }
  return m;
}

}  // namespace sifted::wire
