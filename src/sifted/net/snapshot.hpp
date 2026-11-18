#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/estimation/geodetic.hpp"

// Track snapshot JSON, version 2: the immutable message the publisher sends
// to read-only WebSocket clients (threat model B3). One object per message:
//
//   {"type":"sifted.tracks","version":1,"seq":<n>,"time_ns":<engine time>,
//    "origin":{"lat":..,"lon":..,"alt":..} | null,
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
// on manoeuvre events they are the fading-memory innovation statistic and its
// threshold. sensor and seq are 0 on events not caused by a measurement.
//
// Tracks are listed in id order and extrapolated to time_ns. Numbers use the
// shortest round-trip form, so the text is a deterministic function of the
// engine state (it is part of the replayed output).
namespace sifted::net {

inline constexpr int kSnapshotVersion = 2;

std::string format_snapshot(std::uint64_t seq, Timestamp time,
                            const std::vector<TrackEstimate>& tracks,
                            const std::optional<estimation::GeoOrigin>& origin);

}  // namespace sifted::net
