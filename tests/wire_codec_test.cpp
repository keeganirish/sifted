#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "sifted/wire/codec.hpp"

namespace sifted::wire {
namespace {

using Bytes = std::vector<std::uint8_t>;

Measurement sample() {
  Measurement m;
  m.sensor_id = 7;
  m.kind = SensorKind::BearingOnly;
  m.seq = 3;
  m.time = Timestamp{1'000'000'000};
  m.z = {1.0, 0.0, 0.0};
  m.sigma = {0.5, 0.0, 0.0};
  return m;
}

Bytes read_hex_fixture(const std::string& name) {
  std::ifstream f(std::string(SIFTED_FIXTURE_DIR) + "/" + name);
  const std::string text((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  Bytes out;
  for (std::size_t i = 0; i + 1 < text.size(); i += 2) {
    out.push_back(static_cast<std::uint8_t>(std::stoi(text.substr(i, 2), nullptr, 16)));
  }
  return out;
}

DecodeError error_of(const Bytes& b) {
  const auto r = decode_measurement(b);
  EXPECT_FALSE(r.ok());
  return r.ok() ? DecodeError::Truncated : r.error();
}

}  // namespace

TEST(WireCodec, MatchesGoldenFixture) {
  const Bytes golden = read_hex_fixture("measurement_bearing_only_v1.hex");
  ASSERT_EQ(golden.size(), kMeasurementFrameSize);
  EXPECT_EQ(encode(sample()), golden);  // pins the v1 byte layout
  const auto r = decode_measurement(golden);
  ASSERT_TRUE(r.ok());
  EXPECT_EQ(r.value(), sample());
}

TEST(WireCodec, RoundTripsEveryKind) {
  Measurement rb;
  rb.sensor_id = 65535;
  rb.kind = SensorKind::RangeBearing;
  rb.seq = 0xFFFFFFFFU;
  rb.time = Timestamp{-5};
  rb.z = {1234.5, -3.0, 0.0};
  rb.sigma = {5.0, 0.002, 0.0};
  Measurement geo;
  geo.kind = SensorKind::GeoPosition;
  geo.z = {-33.9, 151.2, 11000.0};
  geo.sigma = {1e-4, 1e-4, 25.0};

  for (const Measurement& m : {sample(), rb, geo}) {
    const Bytes b = encode(m);
    ASSERT_EQ(b.size(), kMeasurementFrameSize);
    const auto r = decode_measurement(b);
    ASSERT_TRUE(r.ok());
    EXPECT_EQ(r.value(), m);
    EXPECT_EQ(encode(r.value()), b);  // canonical: bytes -> value -> same bytes
  }
}

TEST(WireCodec, EncodingIsDeterministic) { EXPECT_EQ(encode(sample()), encode(sample())); }

TEST(WireCodec, RejectsEveryTruncation) {
  const Bytes full = encode(sample());
  for (std::size_t n = 0; n < full.size(); ++n) {
    const Bytes cut(full.begin(), full.begin() + static_cast<std::ptrdiff_t>(n));
    EXPECT_EQ(error_of(cut), DecodeError::Truncated) << "length " << n;
  }
}

TEST(WireCodec, RejectsTrailingBytes) {
  Bytes b = encode(sample());
  b.push_back(0);
  EXPECT_EQ(error_of(b), DecodeError::LengthMismatch);
}

TEST(WireCodec, RejectsHeaderViolations) {
  Bytes b = encode(sample());
  b[0] ^= 0xFF;
  EXPECT_EQ(error_of(b), DecodeError::BadMagic);

  b = encode(sample());
  b[4] = 2;
  EXPECT_EQ(error_of(b), DecodeError::UnsupportedVersion);

  b = encode(sample());
  b[5] = 99;
  EXPECT_EQ(error_of(b), DecodeError::UnknownMessageType);

  b = encode(sample());
  b[6] = 0x01;  // declared payload length 0x0401 = 1025
  b[7] = 0x04;
  EXPECT_EQ(error_of(b), DecodeError::PayloadTooLarge);

  b = encode(sample());
  b[6] = 63;  // wrong size for a Measurement; frame also has a trailing byte
  EXPECT_EQ(error_of(b), DecodeError::LengthMismatch);
}

TEST(WireCodec, HugeDeclaredLengthWithTinyDatagramIsSafe) {
  Bytes b = encode(sample());
  b[6] = 0xFF;
  b[7] = 0xFF;  // declares 65535 bytes
  b.resize(kHeaderSize);
  EXPECT_EQ(error_of(b), DecodeError::PayloadTooLarge);
}

TEST(WireCodec, RejectsPayloadViolations) {
  Bytes b = encode(sample());
  b[kHeaderSize + 3] = 1;  // reserved
  EXPECT_EQ(error_of(b), DecodeError::NonZeroReserved);

  b = encode(sample());
  b[kHeaderSize + 2] = 0;  // kind
  EXPECT_EQ(error_of(b), DecodeError::UnknownSensorKind);

  b = encode(sample());
  b[kHeaderSize + 2] = 4;
  EXPECT_EQ(error_of(b), DecodeError::UnknownSensorKind);

  b = encode(sample());
  for (std::size_t i = 0; i < 8; ++i) {  // z[0] = NaN
    b[kHeaderSize + 16 + i] = i >= 6 ? (i == 6 ? 0xF8 : 0x7F) : 0;
  }
  EXPECT_EQ(error_of(b), DecodeError::InvalidMeasurement);
}

// Flip every byte to every value. Nothing may crash (run under ASan/UBSan in
// CI), and anything accepted must re-encode to exactly the bytes received.
TEST(WireCodec, SingleByteMutationsNeverCrashAndStayCanonical) {
  const Bytes base = encode(sample());
  std::size_t accepted = 0;
  for (std::size_t i = 0; i < base.size(); ++i) {
    for (unsigned v = 0; v < 256; ++v) {
      Bytes b = base;
      b[i] = static_cast<std::uint8_t>(v);
      const auto r = decode_measurement(b);
      if (r.ok()) {
        ++accepted;
        ASSERT_EQ(encode(r.value()), b) << "byte " << i << " value " << v;
      }
    }
  }
  EXPECT_GT(accepted, 0U);
}

TEST(WireCodec, ErrorsHaveNames) {
  EXPECT_EQ(to_string(DecodeError::BadMagic), "bad magic");
  EXPECT_EQ(to_string(DecodeError::InvalidMeasurement), "invalid measurement");
}

}  // namespace sifted::wire
