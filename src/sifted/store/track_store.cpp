#include "sifted/store/track_store.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <utility>

namespace sifted::store {
namespace {

void append_double(std::string& out, double v) {
  if (std::isnan(v)) {
    out += "nan";
    return;
  }
  std::array<char, 32> buf{};
  const auto [ptr, ec] =
      std::to_chars(buf.data(), buf.data() + buf.size(), v, std::chars_format::general, 6);
  (void)ec;  // 32 chars always suffice for 6 significant digits
  out.append(buf.data(), ptr);
}

void append_time(std::string& out, Timestamp t) { out += "t=" + std::to_string(t.ns); }

}  // namespace

std::string_view to_string(TraceEventKind k) noexcept {
  switch (k) {
    case TraceEventKind::Initiated:
      return "initiated";
    case TraceEventKind::Associated:
      return "associated";
    case TraceEventKind::Competed:
      return "competed";
    case TraceEventKind::NearMiss:
      return "near miss";
    case TraceEventKind::UpdateRejected:
      return "update rejected";
    case TraceEventKind::Confirmed:
      return "confirmed";
    case TraceEventKind::Coasting:
      return "coasting";
    case TraceEventKind::Recovered:
      return "recovered";
    case TraceEventKind::Dropped:
      return "dropped";
    case TraceEventKind::ManoeuvreStart:
      return "manoeuvre start";
    case TraceEventKind::ManoeuvreEnd:
      return "manoeuvre end";
  }
  return "unknown";
}

std::string format(const TraceEvent& e) {
  std::string out;
  append_time(out, e.time);
  out += " ";
  out += to_string(e.kind);
  switch (e.kind) {
    case TraceEventKind::Initiated:
    case TraceEventKind::Associated:
    case TraceEventKind::Competed:
    case TraceEventKind::NearMiss:
    case TraceEventKind::UpdateRejected:
      out += " sensor " + std::to_string(e.sensor) + " seq " + std::to_string(e.seq);
      break;
    default:
      break;
  }
  if (!std::isnan(e.distance2)) {
    const bool manoeuvre =
        e.kind == TraceEventKind::ManoeuvreStart || e.kind == TraceEventKind::ManoeuvreEnd;
    out += manoeuvre ? " nis_avg=" : " d2=";
    append_double(out, e.distance2);
    out += manoeuvre ? " threshold=" : " gate=";
    append_double(out, e.gate);
  }
  if (e.kind == TraceEventKind::Competed) {
    out += e.other != 0 ? " (assigned to track " + std::to_string(e.other) + ")"
                        : " (left unassigned)";
  }
  if (e.kind == TraceEventKind::Dropped) {
    out += " (";
    out += association::to_string(e.drop_reason);
    out += ")";
  }
  return out;
}

TrackStore::TrackStore(TraceConfig cfg) : cfg_(cfg), measurements_(cfg.measurement_log) {}

TrackRecord* TrackStore::find(TrackId id) noexcept {
  const auto it = std::lower_bound(live_.begin(), live_.end(), id,
                                   [](const TrackRecord& r, TrackId v) { return r.id < v; });
  return it != live_.end() && it->id == id ? &*it : nullptr;
}

const TrackRecord* TrackStore::find(TrackId id) const noexcept {
  return const_cast<TrackStore*>(this)->find(id);  // NOLINT(*-const-cast)
}

const TrackRecord* TrackStore::find_any(TrackId id) const noexcept {
  if (const TrackRecord* r = find(id)) {
    return r;
  }
  for (const TrackRecord& r : dropped_) {
    if (r.id == id) {
      return &r;
    }
  }
  return nullptr;
}

TrackRecord& TrackStore::add(TrackRecord record) {
  live_.push_back(std::move(record));
  return live_.back();
}

void TrackStore::drop(TrackId id, Timestamp t, association::DropReason reason) {
  const auto it = std::lower_bound(live_.begin(), live_.end(), id,
                                   [](const TrackRecord& r, TrackId v) { return r.id < v; });
  if (it == live_.end() || it->id != id) {
    return;
  }
  it->status = association::TrackStatus::Dropped;
  it->drop_reason = reason;
  it->dropped_at = t;
  TraceEvent e;
  e.kind = TraceEventKind::Dropped;
  e.time = t;
  e.drop_reason = reason;
  record(*it, e);
  if (cfg_.dropped_tracks_kept > 0) {
    dropped_.push_back(std::move(*it));
    while (dropped_.size() > cfg_.dropped_tracks_kept) {
      dropped_.pop_front();
    }
  }
  live_.erase(it);
}

void TrackStore::record(TrackRecord& track, const TraceEvent& e) {
  ++track.totals[static_cast<std::size_t>(e.kind)];
  if (e.kind == TraceEventKind::Initiated) {
    track.initiated = e;
  } else if (e.kind == TraceEventKind::Confirmed && !track.confirmed) {
    track.confirmed = e;
  }
  track.recent.push(e);
}

void TrackStore::credit(TrackRecord& track, SensorId sensor, Timestamp t) {
  auto& s = track.sensors;
  const auto it =
      std::lower_bound(s.begin(), s.end(), sensor,
                       [](const SensorContribution& c, SensorId v) { return c.sensor < v; });
  if (it != s.end() && it->sensor == sensor) {
    ++it->updates;
    it->last = t;
    return;
  }
  if (s.size() >= cfg_.sensors_per_track) {
    ++track.sensors_omitted;
    return;
  }
  s.insert(it, SensorContribution{sensor, 1, t});
}

std::optional<std::string> TrackStore::explain(TrackId id) const {
  const TrackRecord* r = find_any(id);
  if (r == nullptr) {
    return std::nullopt;
  }
  std::string out = "track " + std::to_string(r->id) + " (" +
                    std::string(association::to_string(r->status)) + ")\n";
  out += "  created t=" + std::to_string(r->created.ns) +
         ", last update t=" + std::to_string(r->time.ns) + ", " + std::to_string(r->updates) +
         " updates (" + std::to_string(r->position_hits) + " position-observing)\n";
  if (r->status == association::TrackStatus::Dropped) {
    out += "  dropped t=" + std::to_string(r->dropped_at.ns) + " (" +
           std::string(association::to_string(r->drop_reason)) + ")\n";
  }
  out += "  sensors:";
  if (r->sensors.empty()) {
    out += " none";
  }
  for (const SensorContribution& c : r->sensors) {
    out += " " + std::to_string(c.sensor) + " (" + std::to_string(c.updates) +
           " updates, last t=" + std::to_string(c.last.ns) + ")";
  }
  if (r->sensors_omitted > 0) {
    out += " +" + std::to_string(r->sensors_omitted) + " updates from unlisted sensors";
  }
  out += "\n";
  if (r->initiated) {
    out += "  born: " + format(*r->initiated) + "\n";
  }
  if (r->confirmed) {
    out += "  confirmed: " + format(*r->confirmed) + "\n";
  }
  out += "  recent decisions (" + std::to_string(r->recent.size()) + " of " +
         std::to_string(r->recent.pushed()) + " kept, oldest first):\n";
  for (std::size_t i = 0; i < r->recent.size(); ++i) {
    out += "    " + format(r->recent.at(i)) + "\n";
  }
  out += "  totals:";
  for (std::size_t k = 0; k < kTraceEventKindCount; ++k) {
    if (r->totals[k] > 0) {
      out += " ";
      out += to_string(static_cast<TraceEventKind>(k));
      out += "=" + std::to_string(r->totals[k]);
    }
  }
  out += "\n";
  return out;
}

}  // namespace sifted::store
