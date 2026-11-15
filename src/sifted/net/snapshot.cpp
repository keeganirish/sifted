#include "sifted/net/snapshot.hpp"

#include <array>
#include <charconv>
#include <cmath>

namespace sifted::net {
namespace {

void num(std::string& out, double v) {
  if (!std::isfinite(v)) {
    out += "null";  // JSON has no NaN/Inf; never expected for a committed filter state
    return;
  }
  std::array<char, 32> buf{};
  const auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), v);
  (void)ec;
  out.append(buf.data(), ptr);
}

}  // namespace

std::string format_snapshot(std::uint64_t seq, Timestamp time,
                            const std::vector<TrackEstimate>& tracks,
                            const std::optional<estimation::GeoOrigin>& origin) {
  std::string out = "{\"type\":\"sifted.tracks\",\"version\":" + std::to_string(kSnapshotVersion) +
                    ",\"seq\":" + std::to_string(seq) + ",\"time_ns\":" + std::to_string(time.ns) +
                    ",\"origin\":";
  if (origin) {
    out += "{\"lat\":";
    num(out, origin->lat_deg);
    out += ",\"lon\":";
    num(out, origin->lon_deg);
    out += ",\"alt\":";
    num(out, origin->alt_m);
    out += "}";
  } else {
    out += "null";
  }
  out += ",\"tracks\":[";
  bool first = true;
  for (const TrackEstimate& t : tracks) {
    out += first ? "{" : ",{";
    first = false;
    out += "\"id\":" + std::to_string(t.id) + ",\"status\":\"";
    out += association::to_string(t.status);
    out += "\",\"x\":";
    num(out, t.state(0));
    out += ",\"y\":";
    num(out, t.state(1));
    out += ",\"vx\":";
    num(out, t.state(2));
    out += ",\"vy\":";
    num(out, t.state(3));
    const double a = t.covariance(0, 0);
    const double b = t.covariance(0, 1);
    const double d = t.covariance(1, 1);
    out += ",\"cov\":[";
    num(out, a);
    out += ",";
    num(out, b);
    out += ",";
    num(out, d);
    out += "],\"sigma_m\":";
    num(out, std::sqrt(0.5 * (a + d) + std::sqrt(0.25 * (a - d) * (a - d) + b * b)));
    if (origin) {
      const estimation::Geodetic g =
          estimation::enu_to_geodetic(*origin, Eigen::Vector3d(t.state(0), t.state(1), 0.0));
      out += ",\"lat\":";
      num(out, g.lat_deg);
      out += ",\"lon\":";
      num(out, g.lon_deg);
    }
    out += ",\"updates\":" + std::to_string(t.updates) +
           ",\"last_update_ns\":" + std::to_string(t.last_update.ns) + ",\"sensors\":[";
    for (std::size_t i = 0; i < t.sensors.size(); ++i) {
      out += (i ? "," : "") + std::to_string(t.sensors[i]);
    }
    out += "],\"manoeuvring\":";
    out += t.manoeuvring ? "true" : "false";
    out += "}";
  }
  out += "]}";
  return out;
}

}  // namespace sifted::net
