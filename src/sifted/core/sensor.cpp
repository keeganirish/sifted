#include "sifted/core/sensor.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <system_error>
#include <utility>

namespace sifted {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr std::string_view kHeaderKeyword = "sifted-sensors";
constexpr int kTextVersion = 1;

constexpr std::array<std::pair<SensorKind, std::string_view>, 3> kKindNames{{
    {SensorKind::RangeBearing, "range_bearing"},
    {SensorKind::BearingOnly, "bearing_only"},
    {SensorKind::GeoPosition, "geo_position"},
}};

std::optional<SensorKind> kind_from_name(std::string_view name) noexcept {
  for (const auto& [kind, n] : kKindNames) {
    if (n == name) {
      return kind;
    }
  }
  return std::nullopt;
}

std::string_view kind_name(SensorKind kind) noexcept {
  for (const auto& [k, n] : kKindNames) {
    if (k == kind) {
      return n;
    }
  }
  return "unknown";
}

bool is_space(char c) noexcept { return c == ' ' || c == '\t'; }

// Splits on spaces/tabs. Returns false if there are more than out.size() tokens.
template <std::size_t N>
bool tokenize(std::string_view line, std::array<std::string_view, N>& out, std::size_t& count) {
  count = 0;
  std::size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && is_space(line[i])) {
      ++i;
    }
    if (i == line.size()) {
      break;
    }
    const std::size_t start = i;
    while (i < line.size() && !is_space(line[i])) {
      ++i;
    }
    if (count == N) {
      return false;
    }
    out[count++] = line.substr(start, i - start);
  }
  return true;
}

template <class T>
bool parse_number(std::string_view token, T& out) noexcept {
  const char* first = token.data();
  const char* last = token.data() + token.size();
  const auto [ptr, ec] = std::from_chars(first, last, out);
  return ec == std::errc{} && ptr == last;
}

void append_double(std::string& out, double v) {
  std::array<char, 32> buf{};
  const auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v);
  (void)ec;  // 32 chars always suffice for the shortest form of a double
  out.append(buf.data(), ptr);
}

}  // namespace

std::string_view to_string(SensorTableError e) noexcept {
  switch (e) {
    case SensorTableError::InvalidSensor:
      return "invalid sensor";
    case SensorTableError::DuplicateId:
      return "duplicate sensor id";
    case SensorTableError::TooManySensors:
      return "too many sensors";
    case SensorTableError::TooLarge:
      return "sensor table text too large";
    case SensorTableError::MissingHeader:
      return "missing sifted-sensors header";
    case SensorTableError::UnsupportedVersion:
      return "unsupported sensor table version";
    case SensorTableError::Malformed:
      return "malformed sensor table line";
  }
  return "unknown error";
}

bool is_valid(const SensorConfig& cfg) noexcept {
  if (used_components(cfg.kind) == 0) {
    return false;
  }
  const SensorPose& p = cfg.pose;
  // Comparisons are false for NaN, so NaN is rejected too.
  return std::abs(p.x_m) <= SensorTable::kMaxOffsetM &&
         std::abs(p.y_m) <= SensorTable::kMaxOffsetM && p.yaw_rad >= -kPi && p.yaw_rad <= kPi;
}

std::optional<SensorTableError> SensorTable::add(const SensorConfig& cfg) {
  if (!is_valid(cfg)) {
    return SensorTableError::InvalidSensor;
  }
  const auto pos = std::lower_bound(sensors_.begin(), sensors_.end(), cfg.id,
                                    [](const SensorConfig& s, SensorId id) { return s.id < id; });
  if (pos != sensors_.end() && pos->id == cfg.id) {
    return SensorTableError::DuplicateId;
  }
  if (sensors_.size() >= kMaxSensors) {
    return SensorTableError::TooManySensors;
  }
  sensors_.insert(pos, cfg);
  return std::nullopt;
}

std::optional<std::size_t> SensorTable::index_of(SensorId id) const noexcept {
  const auto pos = std::lower_bound(sensors_.begin(), sensors_.end(), id,
                                    [](const SensorConfig& s, SensorId v) { return s.id < v; });
  if (pos == sensors_.end() || pos->id != id) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(pos - sensors_.begin());
}

const SensorConfig* SensorTable::find(SensorId id) const noexcept {
  const auto i = index_of(id);
  return i ? &sensors_[*i] : nullptr;
}

Result<SensorTable, SensorTableError> parse_sensor_table(std::string_view text) {
  if (text.size() > kMaxSensorTableTextSize) {
    return SensorTableError::TooLarge;
  }

  SensorTable table;
  bool have_header = false;
  while (!text.empty()) {
    const std::size_t nl = text.find('\n');
    std::string_view line = text.substr(0, nl);
    text = nl == std::string_view::npos ? std::string_view{} : text.substr(nl + 1);
    if (!line.empty() && line.back() == '\r') {
      line.remove_suffix(1);  // tolerate CRLF checkouts
    }

    std::array<std::string_view, 6> tok{};
    std::size_t n = 0;
    if (!tokenize(line, tok, n)) {
      return SensorTableError::Malformed;
    }
    if (n == 0 || tok[0].front() == '#') {
      continue;
    }

    if (!have_header) {
      if (tok[0] != kHeaderKeyword || n != 2) {
        return SensorTableError::MissingHeader;
      }
      int version = 0;
      if (!parse_number(tok[1], version)) {
        return SensorTableError::Malformed;
      }
      if (version != kTextVersion) {
        return SensorTableError::UnsupportedVersion;
      }
      have_header = true;
      continue;
    }

    if (tok[0] != "sensor" || n != 6) {
      return SensorTableError::Malformed;
    }
    SensorConfig cfg;
    const auto kind = kind_from_name(tok[2]);
    if (!parse_number(tok[1], cfg.id) || !kind || !parse_number(tok[3], cfg.pose.x_m) ||
        !parse_number(tok[4], cfg.pose.y_m) || !parse_number(tok[5], cfg.pose.yaw_rad)) {
      return SensorTableError::Malformed;
    }
    cfg.kind = *kind;
    if (const auto err = table.add(cfg)) {
      return *err;
    }
  }

  if (!have_header) {
    return SensorTableError::MissingHeader;
  }
  return table;
}

std::string format_sensor_table(const SensorTable& table) {
  std::string out;
  out.append(kHeaderKeyword).append(" ").append(std::to_string(kTextVersion)).append("\n");
  for (const SensorConfig& s : table.sensors()) {
    out.append("sensor ").append(std::to_string(s.id)).append(" ");
    out.append(kind_name(s.kind)).append(" ");
    append_double(out, s.pose.x_m);
    out.append(" ");
    append_double(out, s.pose.y_m);
    out.append(" ");
    append_double(out, s.pose.yaw_rad);
    out.append("\n");
  }
  return out;
}

}  // namespace sifted
