#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "sifted/core/types.hpp"

// Deterministic degraded-network model (ADR 0008).
//
// Turns a sequence of sent frames (send time, bytes) into the sequence the
// receiver would see: (arrival time, bytes), sorted by arrival time. Per
// frame, on that frame's link (its sensor's profile, or the global one):
//
//   * outage:       sent inside a scheduled [start, end) window -> lost;
//   * loss:         a two-state Gilbert-Elliott chain (Gilbert 1960, Elliott
//                   1963). Each frame first steps the chain (Good -> Bad with
//                   probability p_good_to_bad, Bad -> Good with
//                   p_bad_to_good), then is lost with probability `loss` in
//                   Good or `loss_bad` in Bad. p_good_to_bad = 0 gives plain
//                   independent loss;
//   * delay:        delay_ns + a uniform integer in [0, jitter_ns]; when the
//                   jitter exceeds the gap between frames, frames overtake
//                   each other, so reordering is a consequence of variable
//                   delay rather than a separate knob;
//   * duplication:  with probability `duplicate`, a second copy is delivered
//                   with its own independently drawn delay.
//
// Determinism: each link has its own SplitMix64 stream (Steele, Lea and
// Flood, 2014) seeded from (seed, sensor id), and every frame consumes exactly
// five draws whatever happens to it, so the fate of a sensor's frames depends
// only on the seed, its profile and its own frame sequence -- not on other
// sensors, not on how earlier frames fared. Only integer arithmetic and exact
// IEEE comparisons are used, so the Python mirror (sim/sifted_sim/impair.py)
// produces the identical arrival sequence; a shared golden fixture checks it.
//
// This is a test and simulation tool. It never runs inside the engine; the
// live equivalent is the UDP proxy in sim/ (python3 -m sifted_sim.impair).
namespace sifted::net {

class SplitMix64 {
 public:
  explicit SplitMix64(std::uint64_t seed) noexcept : state_(seed) {}
  std::uint64_t next() noexcept;
  // Uniform in [0, 1) with 53 random bits.
  double uniform() noexcept;
  // Uniform integer in [0, n) (Lemire's multiply-shift; n >= 1).
  std::uint64_t below(std::uint64_t n) noexcept;

 private:
  std::uint64_t state_;
};

// [start, end) in send time.
struct Outage {
  std::int64_t start_ns = 0;
  std::int64_t end_ns = 0;
  friend bool operator==(const Outage&, const Outage&) = default;
};

struct LinkProfile {
  double loss = 0.0;           // per-frame loss probability in the Good state
  double p_good_to_bad = 0.0;  // Gilbert-Elliott transition probabilities, per frame
  double p_bad_to_good = 1.0;
  double loss_bad = 1.0;        // per-frame loss probability in the Bad state
  std::int64_t delay_ns = 0;    // fixed one-way delay
  std::int64_t jitter_ns = 0;   // + uniform integer in [0, jitter_ns]
  double duplicate = 0.0;       // probability of one extra copy
  std::vector<Outage> outages;  // this link only

  friend bool operator==(const LinkProfile&, const LinkProfile&) = default;
};

struct ImpairmentConfig {
  std::uint64_t seed = 1;
  LinkProfile global;                                     // every sensor without an override
  std::vector<std::pair<SensorId, LinkProfile>> sensors;  // per-sensor overrides
  std::vector<Outage> outages;                            // every link, in addition

  friend bool operator==(const ImpairmentConfig&, const ImpairmentConfig&) = default;
};

// Probabilities in [0, 1], delays and jitter >= 0 and below one hour,
// outages non-empty intervals, sensor overrides unique.
bool is_valid(const LinkProfile& p) noexcept;
bool is_valid(const ImpairmentConfig& c) noexcept;

struct SentFrame {
  Timestamp send{};
  std::vector<std::uint8_t> bytes;
};

struct ArrivingFrame {
  Timestamp arrival{};
  std::size_t source = 0;  // index of the sent frame it is a copy of
  bool duplicate = false;  // the extra copy
  std::vector<std::uint8_t> bytes;
};

struct ImpairmentStats {
  std::uint64_t sent = 0;
  std::uint64_t lost_outage = 0;
  std::uint64_t lost_good = 0;  // lost while the chain was Good
  std::uint64_t lost_bad = 0;   // lost while the chain was Bad (burst loss)
  std::uint64_t duplicated = 0;
  std::uint64_t delivered = 0;  // arrivals, duplicates included
  std::uint64_t reordered = 0;  // arrivals that come after a copy of a later-sent frame

  friend bool operator==(const ImpairmentStats&, const ImpairmentStats&) = default;
};

// Sensor id of a wire frame (bytes 8-9, little-endian), or 0 if too short.
// The model only reads it to pick the link; it does not validate the frame.
SensorId frame_sensor(std::span<const std::uint8_t> frame) noexcept;

// The fate of one frame on one link. Streaming, so it can also drive a live
// proxy: feed frames in send order and it returns 0, 1 or 2 arrival times.
class Link {
 public:
  Link(std::uint64_t seed, SensorId sensor, const LinkProfile& profile,
       std::span<const Outage> shared_outages);

  struct Fate {
    std::size_t copies = 0;  // 0 (lost), 1, or 2 (duplicated)
    Timestamp arrival[2]{};
  };
  Fate send(Timestamp t, ImpairmentStats* stats = nullptr);

 private:
  SplitMix64 rng_;
  LinkProfile profile_;
  bool bad_ = false;
};

// Seed of the link for `sensor` under run seed `seed`.
std::uint64_t link_seed(std::uint64_t seed, SensorId sensor) noexcept;

// Applies `config` to `frames` (in send order; send times need not be
// sorted). Returns the arrivals sorted by (arrival time, source, duplicate).
// Throws std::invalid_argument if !is_valid(config).
std::vector<ArrivingFrame> impair(std::span<const SentFrame> frames, const ImpairmentConfig& config,
                                  ImpairmentStats* stats = nullptr);

}  // namespace sifted::net
