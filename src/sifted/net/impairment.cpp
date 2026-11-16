#include "sifted/net/impairment.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sifted::net {
namespace {

constexpr std::int64_t kMaxDelayNs = 3'600'000'000'000;  // one hour

bool probability(double p) noexcept { return p >= 0.0 && p <= 1.0; }

bool valid_outages(std::span<const Outage> outages) noexcept {
  return std::all_of(outages.begin(), outages.end(),
                     [](const Outage& o) { return o.start_ns < o.end_ns; });
}

bool in_outage(std::span<const Outage> outages, Timestamp t) noexcept {
  return std::any_of(outages.begin(), outages.end(),
                     [&](const Outage& o) { return o.start_ns <= t.ns && t.ns < o.end_ns; });
}

// t + d without signed overflow (saturates; arrival times near INT64_MAX are
// not meaningful anyway).
Timestamp later(Timestamp t, std::int64_t d) noexcept {
  if (t.ns > INT64_MAX - d) {
    return Timestamp{INT64_MAX};
  }
  return Timestamp{t.ns + d};
}

}  // namespace

std::uint64_t SplitMix64::next() noexcept {
  std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

double SplitMix64::uniform() noexcept { return static_cast<double>(next() >> 11) * 0x1.0p-53; }

std::uint64_t SplitMix64::below(std::uint64_t n) noexcept {
  // High 64 bits of the 128-bit product next() * n, in portable 32-bit pieces.
  const std::uint64_t x = next();
  const std::uint64_t x_lo = x & 0xFFFFFFFFULL;
  const std::uint64_t x_hi = x >> 32;
  const std::uint64_t n_lo = n & 0xFFFFFFFFULL;
  const std::uint64_t n_hi = n >> 32;
  const std::uint64_t lo_lo = x_lo * n_lo;
  const std::uint64_t hi_lo = x_hi * n_lo;
  const std::uint64_t lo_hi = x_lo * n_hi;
  const std::uint64_t hi_hi = x_hi * n_hi;
  const std::uint64_t cross = (lo_lo >> 32) + (hi_lo & 0xFFFFFFFFULL) + (lo_hi & 0xFFFFFFFFULL);
  return hi_hi + (hi_lo >> 32) + (lo_hi >> 32) + (cross >> 32);
}

bool is_valid(const LinkProfile& p) noexcept {
  return probability(p.loss) && probability(p.p_good_to_bad) && probability(p.p_bad_to_good) &&
         probability(p.loss_bad) && probability(p.duplicate) && p.delay_ns >= 0 &&
         p.delay_ns <= kMaxDelayNs && p.jitter_ns >= 0 && p.jitter_ns <= kMaxDelayNs &&
         valid_outages(p.outages);
}

bool is_valid(const ImpairmentConfig& c) noexcept {
  if (!is_valid(c.global) || !valid_outages(c.outages)) {
    return false;
  }
  for (std::size_t i = 0; i < c.sensors.size(); ++i) {
    if (!is_valid(c.sensors[i].second)) {
      return false;
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (c.sensors[j].first == c.sensors[i].first) {
        return false;
      }
    }
  }
  return true;
}

SensorId frame_sensor(std::span<const std::uint8_t> frame) noexcept {
  if (frame.size() < 10) {
    return 0;
  }
  return static_cast<SensorId>(frame[8] | (frame[9] << 8));
}

std::uint64_t link_seed(std::uint64_t seed, SensorId sensor) noexcept {
  return SplitMix64(seed ^ (static_cast<std::uint64_t>(sensor) * 0xD1B54A32D192ED03ULL)).next();
}

Link::Link(std::uint64_t seed, SensorId sensor, const LinkProfile& profile,
           std::span<const Outage> shared_outages)
    : rng_(link_seed(seed, sensor)), profile_(profile) {
  profile_.outages.insert(profile_.outages.end(), shared_outages.begin(), shared_outages.end());
}

Link::Fate Link::send(Timestamp t, ImpairmentStats* stats) {
  // Always exactly five draws, in this order (the Python mirror relies on it).
  const double u_chain = rng_.uniform();
  const double u_loss = rng_.uniform();
  const auto j1 =
      static_cast<std::int64_t>(rng_.below(static_cast<std::uint64_t>(profile_.jitter_ns) + 1));
  const double u_dup = rng_.uniform();
  const auto j2 =
      static_cast<std::int64_t>(rng_.below(static_cast<std::uint64_t>(profile_.jitter_ns) + 1));

  bad_ = bad_ ? !(u_chain < profile_.p_bad_to_good) : u_chain < profile_.p_good_to_bad;
  ImpairmentStats ignored;
  ImpairmentStats& s = stats != nullptr ? *stats : ignored;
  ++s.sent;
  Fate f;
  if (in_outage(profile_.outages, t)) {
    ++s.lost_outage;
    return f;
  }
  if (u_loss < (bad_ ? profile_.loss_bad : profile_.loss)) {
    ++(bad_ ? s.lost_bad : s.lost_good);
    return f;
  }
  f.copies = 1;
  f.arrival[0] = later(t, profile_.delay_ns + j1);
  if (u_dup < profile_.duplicate) {
    f.copies = 2;
    f.arrival[1] = later(t, profile_.delay_ns + j2);
    ++s.duplicated;
  }
  s.delivered += f.copies;
  return f;
}

std::vector<ArrivingFrame> impair(std::span<const SentFrame> frames, const ImpairmentConfig& config,
                                  ImpairmentStats* stats) {
  if (!is_valid(config)) {
    throw std::invalid_argument("sifted::net::impair: invalid ImpairmentConfig");
  }
  std::vector<std::pair<SensorId, Link>> links;  // created on first use, in a fixed order
  auto link_for = [&](SensorId sensor) -> Link& {
    for (auto& [id, link] : links) {
      if (id == sensor) {
        return link;
      }
    }
    const LinkProfile* profile = &config.global;
    for (const auto& [id, p] : config.sensors) {
      if (id == sensor) {
        profile = &p;
      }
    }
    links.emplace_back(sensor, Link(config.seed, sensor, *profile, config.outages));
    return links.back().second;
  };

  ImpairmentStats local;
  std::vector<ArrivingFrame> out;
  out.reserve(frames.size());
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const SentFrame& f = frames[i];
    const Link::Fate fate = link_for(frame_sensor(f.bytes)).send(f.send, &local);
    for (std::size_t c = 0; c < fate.copies; ++c) {
      out.push_back({fate.arrival[c], i, c == 1, f.bytes});
    }
  }
  std::sort(out.begin(), out.end(), [](const ArrivingFrame& a, const ArrivingFrame& b) {
    if (a.arrival != b.arrival) {
      return a.arrival < b.arrival;
    }
    if (a.source != b.source) {
      return a.source < b.source;
    }
    return a.duplicate < b.duplicate;
  });
  std::size_t latest = 0;
  bool any = false;
  for (const ArrivingFrame& a : out) {
    if (any && a.source < latest) {
      ++local.reordered;
    }
    latest = any ? std::max(latest, a.source) : a.source;
    any = true;
  }
  if (stats != nullptr) {
    *stats = local;
  }
  return out;
}

}  // namespace sifted::net
