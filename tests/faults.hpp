#pragma once

// Seeded, general sensor fault injection for tests (M3). Each fault
// transforms a scenario's measurement list (send order) for one sensor and
// leaves every other sensor untouched; the result is re-sorted into send
// order by (time, sensor, original order), as the simulator merges sensors.
// The Python simulator has the same faults for live runs
// (sim/sifted_sim/faults.py); expected system behaviour for each is in
// docs/threat-model.md ("Sensor and input faults").

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <vector>

#include "sifted/core/measurement.hpp"
#include "sifted/net/impairment.hpp"
#include "sifted/wire/codec.hpp"

namespace sifted::testing::faults {

using Measurements = std::vector<Measurement>;

inline Measurements resort(Measurements ms) {
  std::stable_sort(ms.begin(), ms.end(), [](const Measurement& a, const Measurement& b) {
    if (a.time != b.time) {
      return a.time < b.time;
    }
    return a.sensor_id < b.sensor_id;
  });
  return ms;
}

// The sensor emits nothing inside [start, end).
inline Measurements dropout(Measurements ms, SensorId s, std::int64_t start, std::int64_t end) {
  std::erase_if(ms, [&](const Measurement& m) {
    return m.sensor_id == s && m.time.ns >= start && m.time.ns < end;
  });
  return ms;
}

// Intermittent sensor: its output is switched off for whole periods of
// `period_ns`, each independently with probability `p_off` (seeded).
inline Measurements intermittent(Measurements ms, SensorId s, std::int64_t period_ns, double p_off,
                                 std::uint64_t seed) {
  std::map<std::int64_t, bool> off;
  net::SplitMix64 rng(seed);
  std::erase_if(ms, [&](const Measurement& m) {
    if (m.sensor_id != s) {
      return false;
    }
    const std::int64_t slot = m.time.ns / period_ns;
    if (!off.contains(slot)) {
      off[slot] = rng.uniform() < p_off;
    }
    return off[slot];
  });
  return ms;
}

// The sensor restarts at `at`: from then on its seq numbers start again at
// `first_seq` (a reboot), everything else unchanged.
inline Measurements restart(Measurements ms, SensorId s, std::int64_t at,
                            std::uint32_t first_seq = 0) {
  std::uint32_t next = first_seq;
  for (Measurement& m : ms) {
    if (m.sensor_id == s && m.time.ns >= at) {
      m.seq = next++;
    }
  }
  return ms;
}

// Clock fault: the sensor stamps t + offset + drift_ppm * 1e-6 * (t - t0).
// The list stays in send order (the sensor still *sends* at the true time t):
// pair it with the true times, see sent_with_true_times().
inline Measurements clock_fault(Measurements ms, SensorId s, std::int64_t offset_ns,
                                double drift_ppm = 0.0) {
  std::int64_t t0 = INT64_MAX;
  for (const Measurement& m : ms) {
    if (m.sensor_id == s) {
      t0 = std::min(t0, m.time.ns);
    }
  }
  for (Measurement& m : ms) {
    if (m.sensor_id == s) {
      const double drift = drift_ppm * 1e-6 * static_cast<double>(m.time.ns - t0);
      m.time.ns += offset_ns + static_cast<std::int64_t>(drift);
    }
  }
  return ms;
}

// Frames of `stamped` (same length and order as `truth`) sent at the times
// of `truth`: what a sensor with a faulty clock puts on the wire, and when.
inline std::vector<net::SentFrame> sent_with_true_times(const Measurements& truth,
                                                        const Measurements& stamped) {
  std::vector<net::SentFrame> out;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    out.push_back({truth[i].time, wire::encode(stamped[i])});
  }
  return out;
}

// Constant bias added to the measured components (range [m] / bearing [rad]).
inline Measurements bias(Measurements ms, SensorId s, double dz0, double dz1 = 0.0) {
  for (Measurement& m : ms) {
    if (m.sensor_id == s) {
      m.z[0] += dz0;
      if (m.kind == SensorKind::RangeBearing) {
        m.z[1] = std::remainder(m.z[1] + dz1, 2.0 * 3.14159265358979323846);
      }
    }
  }
  return ms;
}

// Stuck (frozen) sensor during [start, end): it keeps sending the last scan
// it produced before `start` -- same values, fresh timestamps and seqs at its
// normal cadence -- instead of what it actually observes.
inline Measurements stuck(Measurements ms, SensorId s, std::int64_t start, std::int64_t end) {
  Measurements frozen;  // the last scan before start
  for (const Measurement& m : ms) {
    if (m.sensor_id == s && m.time.ns < start) {
      if (!frozen.empty() && frozen.front().time != m.time) {
        frozen.clear();
      }
      frozen.push_back(m);
    }
  }
  Measurements out;
  std::uint32_t seq = 1U << 29;  // fresh seqs, as a live (if broken) sensor sends
  std::int64_t last_scan = INT64_MIN;
  for (const Measurement& m : ms) {
    if (m.sensor_id != s || m.time.ns < start || m.time.ns >= end || frozen.empty()) {
      out.push_back(m);
      continue;
    }
    if (m.time.ns == last_scan) {
      continue;  // one frozen scan per real scan time
    }
    last_scan = m.time.ns;
    for (Measurement f : frozen) {
      f.time = m.time;
      f.seq = seq++;
      out.push_back(f);
    }
  }
  return out;
}

// Clutter flood: `per_scan` extra uniform false alarms in every scan of
// range-bearing sensor `s`, inside [0, max_range] (seeded).
inline Measurements clutter(Measurements ms, SensorId s, std::size_t per_scan, double max_range,
                            std::uint64_t seed) {
  net::SplitMix64 rng(seed);
  Measurements out;
  std::uint32_t extra_seq = 1U << 30;  // never collides with the sensor's own seqs
  for (std::size_t i = 0; i < ms.size(); ++i) {
    out.push_back(ms[i]);
    const bool scan_end = ms[i].sensor_id == s && (i + 1 == ms.size() || ms[i + 1].sensor_id != s ||
                                                   ms[i + 1].time != ms[i].time);
    if (!scan_end) {
      continue;
    }
    for (std::size_t k = 0; k < per_scan; ++k) {
      Measurement c = ms[i];
      c.seq = extra_seq++;
      c.z[0] = max_range * std::sqrt(rng.uniform());
      c.z[1] = 2.0 * 3.14159265358979323846 * rng.uniform() - 3.14159265358979323846;
      out.push_back(c);
    }
  }
  return out;
}

}  // namespace sifted::testing::faults
