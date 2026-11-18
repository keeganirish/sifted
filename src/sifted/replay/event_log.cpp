#include "sifted/replay/event_log.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <functional>
#include <istream>
#include <ostream>
#include <system_error>
#include <utility>

#include "sifted/replay/crc32.hpp"

namespace sifted::replay {
namespace {

constexpr std::string_view kHeaderLine = "sifted-log 1";

void put_le(std::vector<std::uint8_t>& out, std::uint64_t v, unsigned n) {
  for (unsigned i = 0; i < n; ++i) {
    out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
  }
}

std::uint64_t get_le(const std::uint8_t* p, unsigned n) noexcept {
  std::uint64_t v = 0;
  for (unsigned i = 0; i < n; ++i) {
    v |= static_cast<std::uint64_t>(p[i]) << (8 * i);
  }
  return v;
}

std::string fmt(double v) {
  std::array<char, 32> buf{};
  const auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v);
  (void)ec;  // 32 chars always suffice for the shortest form of a double
  return std::string(buf.data(), ptr);
}

template <class T>
bool parse_num(std::string_view s, T& out) noexcept {
  const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), out);
  return ec == std::errc{} && ptr == s.data() + s.size() && !s.empty();
}

// Every recorded parameter: its key, how to print it, and how to read it back.
// `optional` marks parameters added after the first recordings were made
// (M3): a header without them is read with their defaults, which reproduce
// the earlier behaviour exactly, so older logs still replay. Writers always
// record every parameter.
struct Param {
  std::string_view key;
  std::function<std::string(const net::PipelineConfig&)> get;
  std::function<bool(net::PipelineConfig&, std::string_view)> set;
  bool optional = false;
};

Param added_later(Param p) {
  p.optional = true;
  return p;
}

Param assignment_method() {
  using association::AssignmentMethod;
  return {
      "assignment",
      [](const net::PipelineConfig& c) {
        return std::string(association::to_string(c.engine.association.method));
      },
      [](net::PipelineConfig& c, std::string_view v) {
        for (const auto m : {AssignmentMethod::GlobalNearestNeighbour, AssignmentMethod::Greedy}) {
          if (v == association::to_string(m)) {
            c.engine.association.method = m;
            return true;
          }
        }
        return false;
      },
      true};
}

Param dbl(std::string_view key, std::function<double&(net::PipelineConfig&)> ref) {
  return {key,
          [ref](const net::PipelineConfig& c) {
            return fmt(ref(const_cast<net::PipelineConfig&>(c)));  // NOLINT(*-const-cast)
          },
          [ref](net::PipelineConfig& c, std::string_view v) { return parse_num(v, ref(c)); }};
}

template <class U>
Param uns(std::string_view key, std::function<U&(net::PipelineConfig&)> ref) {
  return {
      key,
      [ref](const net::PipelineConfig& c) {
        return std::to_string(ref(const_cast<net::PipelineConfig&>(c)));  // NOLINT(*-const-cast)
      },
      [ref](net::PipelineConfig& c, std::string_view v) { return parse_num(v, ref(c)); }};
}

const std::vector<Param>& params() {
  using C = net::PipelineConfig;
  static const std::vector<Param> p = {
      dbl("accel_psd", [](C& c) -> double& { return c.engine.accel_psd; }),
      dbl("initial_velocity_sigma",
          [](C& c) -> double& { return c.engine.initial_velocity_sigma; }),
      dbl("gate_probability",
          [](C& c) -> double& { return c.engine.association.gate_probability; }),
      dbl("near_miss_factor",
          [](C& c) -> double& { return c.engine.association.near_miss_factor; }),
      dbl("manoeuvre_fading", [](C& c) -> double& { return c.engine.manoeuvre.fading; }),
      dbl("manoeuvre_detect_probability",
          [](C& c) -> double& { return c.engine.manoeuvre.detect_probability; }),
      dbl("manoeuvre_accel_psd_factor",
          [](C& c) -> double& { return c.engine.manoeuvre.accel_psd_factor; }),
      uns<std::uint32_t>("confirm_hits",
                         [](C& c) -> std::uint32_t& { return c.engine.lifecycle.confirm_hits; }),
      dbl("tentative_timeout_s",
          [](C& c) -> double& { return c.engine.lifecycle.tentative_timeout_s; }),
      dbl("coast_after_s", [](C& c) -> double& { return c.engine.lifecycle.coast_after_s; }),
      dbl("drop_after_s", [](C& c) -> double& { return c.engine.lifecycle.drop_after_s; }),
      dbl("max_position_sigma_m",
          [](C& c) -> double& { return c.engine.lifecycle.max_position_sigma_m; }),
      uns<std::size_t>("max_tracks", [](C& c) -> std::size_t& { return c.engine.max_tracks; }),
      uns<std::size_t>("trace_events_per_track",
                       [](C& c) -> std::size_t& { return c.engine.trace.events_per_track; }),
      uns<std::size_t>("trace_dropped_tracks_kept",
                       [](C& c) -> std::size_t& { return c.engine.trace.dropped_tracks_kept; }),
      uns<std::size_t>("trace_measurement_log",
                       [](C& c) -> std::size_t& { return c.engine.trace.measurement_log; }),
      uns<std::size_t>("trace_sensors_per_track",
                       [](C& c) -> std::size_t& { return c.engine.trace.sensors_per_track; }),
      dbl("snapshot_interval_s", [](C& c) -> double& { return c.snapshot_interval_s; }),
      uns<std::size_t>("dedup_window", [](C& c) -> std::size_t& { return c.dedup_window; }),
      uns<std::size_t>("max_scan_size", [](C& c) -> std::size_t& { return c.max_scan_size; }),
      // Added in M3 (optional when reading; see Param).
      assignment_method(),
      added_later(dbl("reorder_window_s", [](C& c) -> double& { return c.reorder_window_s; })),
      added_later(uns<std::size_t>("reorder_capacity",
                                   [](C& c) -> std::size_t& { return c.reorder_capacity; })),
      added_later(dbl("max_time_ahead_s", [](C& c) -> double& { return c.max_time_ahead_s; })),
      // Display only: an older log replays the same tracks, with the default excerpt.
      added_later(uns<std::size_t>("snapshot_trace_events",
                                   [](C& c) -> std::size_t& { return c.snapshot_trace_events; })),
  };
  return p;
}

std::vector<std::string_view> split(std::string_view line) {
  std::vector<std::string_view> out;
  std::size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && line[i] == ' ') {
      ++i;
    }
    const std::size_t start = i;
    while (i < line.size() && line[i] != ' ') {
      ++i;
    }
    if (i > start) {
      out.push_back(line.substr(start, i - start));
    }
  }
  return out;
}

bool read_exact(std::istream& in, std::uint8_t* dst, std::size_t n) {
  in.read(reinterpret_cast<char*>(dst),
          static_cast<std::streamsize>(n));  // NOLINT(*-reinterpret-cast)
  return static_cast<std::size_t>(in.gcount()) == n;
}

}  // namespace

std::string_view to_string(LogError e) noexcept {
  switch (e) {
    case LogError::Io:
      return "i/o error";
    case LogError::BadMagic:
      return "not a sifted event log (bad magic)";
    case LogError::UnsupportedVersion:
      return "unsupported event log version";
    case LogError::NonZeroReserved:
      return "non-zero reserved field";
    case LogError::HeaderTooLarge:
      return "event log header too large";
    case LogError::BadHeader:
      return "malformed event log header";
    case LogError::BadChecksum:
      return "checksum mismatch";
    case LogError::Truncated:
      return "truncated event log";
    case LogError::BadRecord:
      return "malformed record";
    case LogError::TimeWentBackwards:
      return "receive time went backwards";
  }
  return "unknown error";
}

std::string format_header(const RunHeader& h) {
  std::string out(kHeaderLine);
  out += "\n";
  for (const Param& p : params()) {
    out.append("param ").append(p.key).append(" ").append(p.get(h.config)).append("\n");
  }
  if (const auto& o = h.config.engine.origin) {
    out += "origin " + fmt(o->lat_deg) + " " + fmt(o->lon_deg) + " " + fmt(o->alt_m) + "\n";
  }
  out += "sensors\n";
  out += format_sensor_table(h.sensors);
  return out;
}

Result<RunHeader, LogError> parse_header(std::string_view text) {
  if (text.size() > kMaxHeaderSize) {
    return LogError::HeaderTooLarge;
  }
  RunHeader h;
  std::vector<char> seen(params().size(), 0);
  bool have_first = false;
  bool have_origin = false;
  while (!text.empty()) {
    const std::size_t nl = text.find('\n');
    if (nl == std::string_view::npos) {
      return LogError::BadHeader;  // every header line ends with '\n'
    }
    const std::string_view line = text.substr(0, nl);
    text.remove_prefix(nl + 1);
    if (!have_first) {
      if (line != kHeaderLine) {
        return LogError::BadHeader;
      }
      have_first = true;
      continue;
    }
    if (line == "sensors") {
      const auto table = parse_sensor_table(text);
      // The table must be in canonical form, so the header has one encoding.
      if (!table.ok() || format_sensor_table(table.value()) != text) {
        return LogError::BadHeader;
      }
      h.sensors = table.value();
      for (std::size_t i = 0; i < seen.size(); ++i) {
        if (seen[i] == 0 && !params()[i].optional) {
          return LogError::BadHeader;  // a required parameter is missing
        }
      }
      if (!net::is_valid(h.config)) {
        return LogError::BadHeader;
      }
      return h;
    }
    const auto tok = split(line);
    if (tok.size() == 4 && tok[0] == "origin" && !have_origin) {
      estimation::GeoOrigin o;
      if (!parse_num(tok[1], o.lat_deg) || !parse_num(tok[2], o.lon_deg) ||
          !parse_num(tok[3], o.alt_m)) {
        return LogError::BadHeader;
      }
      h.config.engine.origin = o;
      have_origin = true;
      continue;
    }
    if (tok.size() != 3 || tok[0] != "param") {
      return LogError::BadHeader;
    }
    const auto& ps = params();
    const auto it =
        std::find_if(ps.begin(), ps.end(), [&](const Param& p) { return p.key == tok[1]; });
    if (it == ps.end()) {
      return LogError::BadHeader;  // unknown key: a v1 reader does not guess
    }
    const auto idx = static_cast<std::size_t>(it - ps.begin());
    if (seen[idx] != 0 || !it->set(h.config, tok[2])) {
      return LogError::BadHeader;
    }
    seen[idx] = 1;
  }
  return LogError::BadHeader;  // no "sensors" section
}

std::vector<std::uint8_t> encode_header(const RunHeader& h) {
  const std::string text = format_header(h);
  std::vector<std::uint8_t> out(kLogMagic.begin(), kLogMagic.end());
  put_le(out, kLogVersion, 2);
  put_le(out, 0, 2);
  put_le(out, text.size(), 4);
  out.insert(out.end(), text.begin(), text.end());
  const std::span<const std::uint8_t> body(out.data() + kPreambleSize, text.size());
  put_le(out, crc32(body), 4);
  return out;
}

std::vector<std::uint8_t> encode_record(RecordType type, Timestamp recv,
                                        std::span<const std::uint8_t> payload) {
  std::vector<std::uint8_t> out;
  out.reserve(kRecordOverhead + payload.size());
  out.push_back(static_cast<std::uint8_t>(type));
  out.push_back(0);
  put_le(out, payload.size(), 2);
  put_le(out, static_cast<std::uint64_t>(recv.ns), 8);
  out.insert(out.end(), payload.begin(), payload.end());
  put_le(out, crc32(out), 4);
  return out;
}

EventLogWriter::EventLogWriter(std::ostream& out, const RunHeader& header) : out_(&out) {
  const auto bytes = encode_header(header);
  out_->write(reinterpret_cast<const char*>(bytes.data()),  // NOLINT(*-reinterpret-cast)
              static_cast<std::streamsize>(bytes.size()));
}

bool EventLogWriter::write(RecordType type, Timestamp recv, std::span<const std::uint8_t> payload) {
  if (payload.size() > kMaxRecordPayload || (last_ && recv < *last_)) {
    return false;
  }
  const auto bytes = encode_record(type, recv, payload);
  out_->write(reinterpret_cast<const char*>(bytes.data()),  // NOLINT(*-reinterpret-cast)
              static_cast<std::streamsize>(bytes.size()));
  last_ = recv;
  ++records_;
  return out_->good();
}

bool EventLogWriter::write_frame(Timestamp recv, std::span<const std::uint8_t> frame) {
  return write(RecordType::Frame, recv, frame);
}

bool EventLogWriter::write_tick(Timestamp recv) { return write(RecordType::Tick, recv, {}); }

bool EventLogWriter::good() const { return out_->good(); }

Result<EventLogReader, LogError> EventLogReader::open(std::istream& in) {
  std::array<std::uint8_t, kPreambleSize> pre{};
  if (!read_exact(in, pre.data(), pre.size())) {
    return LogError::Truncated;
  }
  if (!std::equal(kLogMagic.begin(), kLogMagic.end(), pre.begin())) {
    return LogError::BadMagic;
  }
  if (get_le(pre.data() + 8, 2) != kLogVersion) {
    return LogError::UnsupportedVersion;
  }
  if (get_le(pre.data() + 10, 2) != 0) {
    return LogError::NonZeroReserved;
  }
  const std::uint64_t len = get_le(pre.data() + 12, 4);
  if (len > kMaxHeaderSize) {
    return LogError::HeaderTooLarge;  // checked before allocating
  }
  std::vector<std::uint8_t> text(static_cast<std::size_t>(len) + 4);
  if (!read_exact(in, text.data(), text.size())) {
    return LogError::Truncated;
  }
  const std::span<const std::uint8_t> body(text.data(), static_cast<std::size_t>(len));
  if (crc32(body) != get_le(text.data() + len, 4)) {
    return LogError::BadChecksum;
  }
  auto h = parse_header(std::string_view(reinterpret_cast<const char*>(body.data()),  // NOLINT
                                         body.size()));
  if (!h.ok()) {
    return h.error();
  }
  return EventLogReader(in, h.value());
}

Result<std::optional<LogRecord>, LogError> EventLogReader::next() {
  std::array<std::uint8_t, 12> head{};
  in_->read(reinterpret_cast<char*>(head.data()), 1);  // NOLINT(*-reinterpret-cast)
  if (in_->gcount() == 0) {
    return std::optional<LogRecord>{};  // clean end of file
  }
  if (!read_exact(*in_, head.data() + 1, head.size() - 1)) {
    return LogError::Truncated;
  }
  const auto type = head[0];
  const std::uint64_t len = get_le(head.data() + 2, 2);
  if (head[1] != 0) {
    return LogError::NonZeroReserved;
  }
  const bool frame = type == static_cast<std::uint8_t>(RecordType::Frame);
  const bool tick = type == static_cast<std::uint8_t>(RecordType::Tick);
  if (!(frame && len <= kMaxRecordPayload && len > 0) && !(tick && len == 0)) {
    return LogError::BadRecord;
  }
  std::vector<std::uint8_t> rest(static_cast<std::size_t>(len) + 4);
  if (!read_exact(*in_, rest.data(), rest.size())) {
    return LogError::Truncated;
  }
  std::uint32_t crc = crc32(head);
  crc = crc32(std::span<const std::uint8_t>(rest.data(), static_cast<std::size_t>(len)), crc);
  if (crc != get_le(rest.data() + len, 4)) {
    return LogError::BadChecksum;
  }
  LogRecord r;
  r.type = static_cast<RecordType>(type);
  r.recv = Timestamp{static_cast<std::int64_t>(get_le(head.data() + 4, 8))};
  if (last_ && r.recv < *last_) {
    return LogError::TimeWentBackwards;
  }
  last_ = r.recv;
  rest.resize(static_cast<std::size_t>(len));
  r.payload = std::move(rest);
  return std::optional<LogRecord>(std::move(r));
}

}  // namespace sifted::replay
