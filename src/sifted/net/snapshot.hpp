#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/estimation/geodetic.hpp"

// Track snapshot JSON, version 1: the immutable message the publisher sends
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
//               "sensors":[<id>,...],"manoeuvring":true|false}, ...]}
//
// Tracks are listed in id order and extrapolated to time_ns. Numbers use the
// shortest round-trip form, so the text is a deterministic function of the
// engine state (it is part of the replayed output).
namespace sifted::net {

inline constexpr int kSnapshotVersion = 1;

std::string format_snapshot(std::uint64_t seq, Timestamp time,
                            const std::vector<TrackEstimate>& tracks,
                            const std::optional<estimation::GeoOrigin>& origin);

}  // namespace sifted::net
