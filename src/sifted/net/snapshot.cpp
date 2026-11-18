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

// The kind and drop reason come from closed sets of plain words, so they need
// no JSON escaping. A NaN statistic (none defined for this event) becomes null.
void trace_event(std::string& out, const store::TraceEvent& e) {
  out += "{\"kind\":\"";
  out += store::to_string(e.kind);
  out += "\",\"time_ns\":" + std::to_string(e.time.ns) + ",\"sensor\":" + std::to_string(e.sensor) +
         ",\"seq\":" + std::to_string(e.seq) + ",\"d2\":";
  num(out, e.distance2);
  out += ",\"gate\":";
  num(out, e.gate);
  out += ",\"other\":" + std::to_string(e.other) + ",\"drop_reason\":\"";
  out += association::to_string(e.drop_reason);
  out += "\"}";
}

void opt_int(std::string& out, const std::optional<std::int64_t>& v) {
  out += v ? std::to_string(*v) : "null";
}

void feed_status(std::string& out, const FeedStatus& f) {
  out += ",\"feed\":{\"state\":\"";
  out += to_string(f.state);
  out += "\",\"age_ms\":";
  opt_int(out, f.age_ms);
  out += ",\"predicted_ms\":" + std::to_string(f.predicted_ms) + ",\"sensors\":[";
  for (std::size_t i = 0; i < f.sensors.size(); ++i) {
    const SensorFeed& s = f.sensors[i];
    out += (i > 0 ? ",{\"id\":" : "{\"id\":") + std::to_string(s.id) + ",\"state\":\"";
    out += to_string(s.state);
    out += "\",\"age_ms\":";
    opt_int(out, s.age_ms);
    out += ",\"stale_after_ms\":" + std::to_string(s.stale_after_ms) + "}";
  }
  out += "]}";
}

}  // namespace

std::string_view to_string(FeedState s) noexcept {
  switch (s) {
    case FeedState::Waiting:
      return "waiting";
    case FeedState::Live:
      return "live";
    case FeedState::Degraded:
      return "degraded";
    case FeedState::Stale:
      return "stale";
  }
  return "unknown";
}

std::string format_snapshot(std::uint64_t seq, Timestamp time,
                            const std::vector<TrackEstimate>& tracks,
                            const std::optional<estimation::GeoOrigin>& origin,
                            const std::optional<FeedStatus>& feed) {
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
  if (feed) {
    feed_status(out, *feed);
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
    out += ",\"trace_total\":" + std::to_string(t.trace_total) + ",\"trace\":[";
    for (std::size_t i = 0; i < t.trace.size(); ++i) {
      if (i > 0) {
        out += ",";
      }
      trace_event(out, t.trace[i]);
    }
    out += "]}";
  }
  out += "]}";
  return out;
}

}  // namespace sifted::net
