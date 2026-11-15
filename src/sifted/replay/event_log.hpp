#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/core/result.hpp"
#include "sifted/core/sensor.hpp"
#include "sifted/core/types.hpp"
#include "sifted/net/pipeline_config.hpp"
#include "sifted/wire/codec.hpp"

// Append-only event log of everything the engine consumed, in the order it
// consumed it, so a run can be replayed bit for bit. Normative description:
// docs/adr/0005-event-log.md.
//
//   file    := preamble header_text header_crc record*
//   preamble:= magic "SIFTLOG\0" (8) | version u16 (=1) | reserved u16 (=0)
//              | header_len u32 (<= kMaxHeaderSize)
//   header_crc := CRC-32 of header_text, u32
//   record  := type u8 | reserved u8 (=0) | len u16 | recv_ns i64 | payload[len]
//              | crc u32 (CRC-32 of type..payload)
//
// Little-endian throughout. Record types: 1 = Frame (payload: one wire frame,
// exactly as received, len <= wire::kHeaderSize + wire::kMaxPayloadSize);
// 2 = Tick (len 0: the live engine thread flushed a pending scan because the
// input went quiet). recv_ns never decreases from one record to the next.
//
// The header text carries the format version, every run parameter that
// affects output (PipelineConfig, including the tangent-plane origin) and the
// canonical sensor table text (ADR 0003), so a log replays on its own.
//
// The log is semi-trusted input (threat model B4): the reader checks sizes
// before allocating, checksums, reserved fields, record types and time order,
// and frames still go through the strict wire decoder during replay.
namespace sifted::replay {

inline constexpr std::array<std::uint8_t, 8> kLogMagic{'S', 'I', 'F', 'T', 'L', 'O', 'G', 0};
inline constexpr std::uint16_t kLogVersion = 1;
inline constexpr std::size_t kPreambleSize = 16;
inline constexpr std::size_t kMaxHeaderSize = 128 * 1024;
inline constexpr std::size_t kRecordOverhead = 1 + 1 + 2 + 8 + 4;
inline constexpr std::size_t kMaxRecordPayload = wire::kHeaderSize + wire::kMaxPayloadSize;

enum class RecordType : std::uint8_t { Frame = 1, Tick = 2 };

enum class LogError : std::uint8_t {
  Io,
  BadMagic,
  UnsupportedVersion,
  NonZeroReserved,
  HeaderTooLarge,
  BadHeader,  // header text fails to parse, misses or repeats a key, or bad sensor table
  BadChecksum,
  Truncated,  // file ends inside the preamble, header or a record
  BadRecord,  // unknown type or a length the type does not allow
  TimeWentBackwards,
};

std::string_view to_string(LogError e) noexcept;

// What a log needs to rebuild the run: the sensor table and the parameters.
struct RunHeader {
  SensorTable sensors;
  net::PipelineConfig config;

  friend bool operator==(const RunHeader&, const RunHeader&) = default;
};

// Header text ("sifted-log 1", one "param <key> <value>" per parameter, an
// optional "origin <lat> <lon> <alt>", then "sensors" followed by the
// canonical sensor table text). Numbers in shortest round-trip form.
std::string format_header(const RunHeader& h);
Result<RunHeader, LogError> parse_header(std::string_view text);

struct LogRecord {
  RecordType type = RecordType::Frame;
  Timestamp recv{};
  std::vector<std::uint8_t> payload;
};

// Bytes of a complete preamble + header, and of one record.
std::vector<std::uint8_t> encode_header(const RunHeader& h);
std::vector<std::uint8_t> encode_record(RecordType type, Timestamp recv,
                                        std::span<const std::uint8_t> payload);

// Writes a log to an ostream. Enforces the same invariants the reader checks
// (payload size, non-decreasing recv time) and refuses anything else.
class EventLogWriter {
 public:
  EventLogWriter(std::ostream& out, const RunHeader& header);
  // False if the record would violate the format; nothing is written then.
  bool write_frame(Timestamp recv, std::span<const std::uint8_t> frame);
  bool write_tick(Timestamp recv);
  bool good() const;
  std::uint64_t records() const noexcept { return records_; }

 private:
  bool write(RecordType type, Timestamp recv, std::span<const std::uint8_t> payload);
  std::ostream* out_;
  std::optional<Timestamp> last_;
  std::uint64_t records_ = 0;
};

class EventLogReader {
 public:
  // Reads and validates the preamble and header.
  static Result<EventLogReader, LogError> open(std::istream& in);

  const RunHeader& header() const noexcept { return header_; }

  // The next record, nullopt at a clean end of file, or the error that makes
  // the rest of the log unusable.
  Result<std::optional<LogRecord>, LogError> next();

 private:
  EventLogReader(std::istream& in, RunHeader h) : in_(&in), header_(std::move(h)) {}
  std::istream* in_;
  RunHeader header_;
  std::optional<Timestamp> last_;
};

}  // namespace sifted::replay
