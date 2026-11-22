#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/estimation/geodetic.hpp"

// Track snapshot JSON, version 2: the immutable message the publisher sends
// to read-only WebSocket clients (threat model B3). One object per message:
//
//   {"type":"sifted.tracks","version":2,"seq":<n>,"time_ns":<engine time>,
//    "origin":{"lat":..,"lon":..,"alt":..} | null,
//    "feed":{"state":"waiting|live|degraded|stale",       // only with feed staleness on
//            "age_ms":<n>|null,"predicted_ms":<n>,
//            "sensors":[{"id":<n>,"state":"waiting|live|stale",
//                        "age_ms":<n>|null,"stale_after_ms":<n>}, ...]},
//    "tracks":[{"id":<n>,"status":"tentative|confirmed|coasting",
//               "x":..,"y":..,"vx":..,"vy":..,      // tracking frame, m and m/s
//               "cov":[pxx,pxy,pyy],                 // position covariance, m^2
//               "sigma_m":..,                         // sqrt of its largest eigenvalue
//               "lat":..,"lon":..,                    // only when an origin is set
//               "updates":<n>,"last_update_ns":<t>,
//               "sensors":[<id>,...],"manoeuvring":true|false,
//               "trace_total":<n>,                    // decisions ever recorded
//               "trace":[{"kind":"<store::to_string(TraceEventKind)>",
//                         "time_ns":<t>,"sensor":<id>,"seq":<n>,
//                         "d2":<x>|null,"gate":<x>|null,
//                         "other":<track id>,"drop_reason":"<reason>"}, ...]}, ...]}
//
// `trace` holds the track's newest decisions, oldest first, at most
// PipelineConfig::snapshot_trace_events of them; its last entry is decision
// number trace_total - 1. d2 and gate are null for events with no statistic;
// on "manoeuvre start/end" (legacy constant-velocity filter) they are the
// fading-memory innovation statistic and its threshold, on "manoeuvre
// likely/unlikely" (IMM, ADR 0014) the probability of the manoeuvre modes and
// its threshold. sensor and seq are 0 on events not caused by a measurement.
// The two IMM kinds are additive (kind is open text to a reader, and runs with
// the legacy filter never emit them), so the version stays 2. Mode
// probabilities themselves are not published: `manoeuvring` and these events
// carry what an operator acts on (ADR 0014).
//
// `feed` says how fresh the input is, in receive time: age_ms since the newest
// accepted frame, per sensor and overall. predicted_ms is how far time_ns lies
// beyond the last accepted measurement: non-zero only in a snapshot published
// during total silence, whose tracks are predictions. The field is additive,
// so the version stays 2 and recordings without it replay byte for byte.
//
// Tracks are listed in id order and extrapolated to time_ns. Numbers use the
// shortest round-trip form, so the text is a deterministic function of the
// engine state (it is part of the replayed output).
namespace sifted::net {

inline constexpr int kSnapshotVersion = 2;

enum class FeedState : std::uint8_t { Waiting, Live, Degraded, Stale };

std::string_view to_string(FeedState s) noexcept;

struct SensorFeed {
  SensorId id = 0;
  FeedState state = FeedState::Waiting;  // Waiting, Live or Stale
  std::optional<std::int64_t> age_ms;    // none until the sensor has reported
  std::int64_t stale_after_ms = 0;

  friend bool operator==(const SensorFeed&, const SensorFeed&) = default;
};

struct FeedStatus {
  FeedState state = FeedState::Waiting;
  std::optional<std::int64_t> age_ms;
  std::int64_t predicted_ms = 0;
  std::vector<SensorFeed> sensors;  // in sensor table order
};

std::string format_snapshot(std::uint64_t seq, Timestamp time,
                            const std::vector<TrackEstimate>& tracks,
                            const std::optional<estimation::GeoOrigin>& origin,
                            const std::optional<FeedStatus>& feed = std::nullopt);

}  // namespace sifted::net
