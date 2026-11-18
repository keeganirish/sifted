// Structure-aware libFuzzer target for the engine and the ingest pipeline.
//
// The input is read as a small configuration followed by a sequence of
// measurements, valid and invalid: sensor ids known and unknown, kinds that
// match and that do not, repeated and reset seq numbers, times that go
// forwards, repeat and go backwards, plausible values and raw bit patterns
// (NaN, Inf, huge, denormal). They are fed two ways:
//
//   A. straight into two Engines, in scans of input-chosen size;
//   B. as encoded frames into two net::Pipelines with input-chosen receive
//      times, ticks, reorder window, clock gate, timeline guard and feed
//      status (the daemon path). Every frame must end up in exactly one of:
//      the engine, duplicates, clock gate, timeline guard, decode errors.
//
// Invariants checked on every input:
//   1. no crash, hang, out-of-bounds access or UB (ASan/UBSan);
//   2. every measurement gets exactly one outcome: one result per input, the
//      per-outcome counters sum to measurements_processed;
//   3. live tracks never exceed max_tracks, after every scan;
//   4. every bounded buffer stays bounded: per-track trace windows, the
//      contributing-sensor lists, the dropped-track history, the measurement
//      log, and the pipeline's reorder stage;
//   5. every live track's state and covariance are finite;
//   6. determinism: the two engines (and the two pipelines) fed the same
//      input produce identical results, tracks and output bytes. The second
//      of each pair runs with the gate screen off (EngineConfig::association
//      .gate_screen), so this also proves the screen never changes a result.
//
// Run:  mkdir -p build/fuzz-work-engine && build/fuzz/fuzz/fuzz_engine
//       build/fuzz-work-engine fuzz/corpus_engine -max_total_time=60

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "sifted/core/engine.hpp"
#include "sifted/net/ingest.hpp"
#include "sifted/wire/codec.hpp"

namespace {

using namespace sifted;

void check(bool ok) {
  if (!ok) {
    std::abort();
  }
}

class Reader {
 public:
  Reader(const std::uint8_t* d, std::size_t n) : d_(d), n_(n) {}
  bool empty() const { return i_ >= n_; }
  std::uint8_t u8() { return i_ < n_ ? d_[i_++] : 0; }
  std::uint64_t u64() {
    std::uint64_t v = 0;
    for (int k = 0; k < 8; ++k) {
      v = (v << 8) | u8();
    }
    return v;
  }
  double raw_double() {
    const std::uint64_t bits = u64();
    double v = 0.0;
    std::memcpy(&v, &bits, sizeof v);
    return v;
  }
  // A value in [lo, hi] most of the time, a raw bit pattern sometimes.
  double value(double lo, double hi) {
    const std::uint8_t mode = u8();
    if (mode < 16) {
      return raw_double();
    }
    return lo + (hi - lo) * static_cast<double>(u8() | (u8() << 8)) / 65535.0;
  }

 private:
  const std::uint8_t* d_;
  std::size_t n_;
  std::size_t i_ = 0;
};

SensorTable table() {
  SensorTable t;
  (void)t.add({1, SensorKind::RangeBearing, {0.0, 0.0, 0.0}});
  (void)t.add({2, SensorKind::BearingOnly, {-3000.0, 2000.0, -0.6}});
  (void)t.add({3, SensorKind::GeoPosition, {0.0, 0.0, 0.0}});
  (void)t.add({4, SensorKind::RangeBearing, {5000.0, -1000.0, 2.5}});
  return t;
}

constexpr SensorId kSensors[] = {1, 2, 3, 4, 77};  // 77: not in the table
constexpr SensorKind kKinds[] = {SensorKind::RangeBearing, SensorKind::BearingOnly,
                                 SensorKind::GeoPosition};

Measurement next_measurement(Reader& r, std::int64_t& t, std::uint32_t& seq) {
  Measurement m;
  const std::uint8_t sel = r.u8();
  m.sensor_id = kSensors[sel % 5];
  // Usually the sensor's own kind; sometimes a mismatch.
  m.kind = (sel & 0x80U) != 0 ? kKinds[(sel >> 3) % 3]
           : m.sensor_id == 2 ? SensorKind::BearingOnly
           : m.sensor_id == 3 ? SensorKind::GeoPosition
                              : SensorKind::RangeBearing;
  const std::uint8_t tm = r.u8();
  if (tm == 0xFF) {
    t = static_cast<std::int64_t>(r.u64());  // anything, including extremes
  } else if (tm >= 0x10) {
    // Mostly forward, sometimes the same time (a scan), sometimes backwards.
    // Wrapping (two's complement) so extreme times do not overflow here;
    // the engine and pipeline must cope with whatever results.
    const std::int64_t step = (static_cast<std::int64_t>(tm) - 0x30) * 50'000'000;
    t = static_cast<std::int64_t>(static_cast<std::uint64_t>(t) + static_cast<std::uint64_t>(step));
  }
  m.time = Timestamp{t};
  const std::uint8_t sq = r.u8();
  seq = sq < 0x20 ? sq : seq + 1;  // small values repeat and reset
  m.seq = seq;
  switch (m.kind) {
    case SensorKind::RangeBearing:
      m.z = {r.value(0.0, 20000.0), r.value(-3.2, 3.2), 0.0};
      m.sigma = {r.value(0.1, 100.0), r.value(1e-4, 0.1), 0.0};
      break;
    case SensorKind::BearingOnly:
      m.z = {r.value(-3.2, 3.2), 0.0, 0.0};
      m.sigma = {r.value(1e-4, 0.1), 0.0, 0.0};
      break;
    case SensorKind::GeoPosition:
      m.z = {r.value(51.0, 52.0), r.value(-1.0, 1.0), r.value(-100.0, 12000.0)};
      m.sigma = {r.value(1.0, 500.0), r.value(1.0, 500.0), r.value(1.0, 500.0)};
      break;
  }
  return m;
}

void check_bounds(const Engine& e) {
  const EngineConfig& c = e.config();
  check(e.store().size() <= c.max_tracks);
  check(e.store().dropped().size() <= c.trace.dropped_tracks_kept);
  check(e.store().measurements().size() <= c.trace.measurement_log);
  for (const auto& r : e.store().live()) {
    check(r.recent.size() <= c.trace.events_per_track);
    check(r.sensors.size() <= c.trace.sensors_per_track);
    check(r.filter.state().allFinite() && r.filter.covariance().allFinite());
  }
}

void check_counts(const Engine& e, std::size_t offered) {
  std::uint64_t sum = 0;
  for (std::size_t o = 0; o < kMeasurementOutcomeCount; ++o) {
    sum += e.count(static_cast<MeasurementOutcome>(o));
  }
  check(sum == e.measurements_processed());
  check(e.measurements_processed() == offered);
}

bool same_tracks(const Engine& a, const Engine& b) {
  const auto ta = a.tracks();
  const auto tb = b.tracks();
  if (ta.size() != tb.size()) {
    return false;
  }
  for (std::size_t i = 0; i < ta.size(); ++i) {
    if (ta[i].id != tb[i].id || ta[i].status != tb[i].status || ta[i].time != tb[i].time ||
        !(ta[i].state == tb[i].state) || !(ta[i].covariance == tb[i].covariance)) {
      return false;
    }
  }
  return true;
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
  Reader r(data, size);
  net::PipelineConfig pc;
  EngineConfig& c = pc.engine;
  c.max_tracks = 1 + r.u8() % 24;
  c.trace.events_per_track = r.u8() % 8;
  c.trace.measurement_log = r.u8() % 16;
  c.trace.dropped_tracks_kept = r.u8() % 6;
  c.trace.sensors_per_track = 1 + r.u8() % 4;
  c.lifecycle.confirm_hits = 1 + r.u8() % 4;
  c.association.method = (r.u8() & 1U) != 0 ? association::AssignmentMethod::Greedy
                                            : association::AssignmentMethod::GlobalNearestNeighbour;
  c.origin = estimation::GeoOrigin{51.5, 0.0, 0.0};
  pc.max_scan_size = 1 + r.u8() % 16;
  pc.dedup_window = r.u8() % 8;
  pc.reorder_window_s = (r.u8() % 8) * 0.25;
  pc.reorder_capacity = 1 + r.u8() % 16;
  pc.max_time_ahead_s = (r.u8() % 4) * 0.5;
  pc.snapshot_interval_s = (r.u8() % 4) * 0.5;
  pc.max_time_jump_s = (r.u8() % 4) * 1.5;
  pc.timeline_confirm_frames = 2 + r.u8() % 3;
  pc.timeline_confirm_s = (r.u8() % 4) * 0.5;
  pc.feed_stale_factor = (r.u8() % 3) * 1.5;
  pc.feed_stale_min_s = (r.u8() % 4) * 0.5;
  if (!net::is_valid(pc)) {
    return 0;  // e.g. a trace size of 0 that the store rejects: not this target's business
  }

  struct Item {
    Measurement m;
    std::uint8_t scan_break;
    std::uint8_t recv_step;
  };
  std::vector<Item> items;
  std::int64_t t = 1'000'000'000;
  std::uint32_t seq = 0;
  while (!r.empty() && items.size() < 512) {
    Item it;
    it.m = next_measurement(r, t, seq);
    it.scan_break = r.u8();
    it.recv_step = r.u8();
    items.push_back(it);
  }

  // ---- A: the engine directly -------------------------------------------------
  EngineConfig unscreened = c;
  unscreened.association.gate_screen = false;
  Engine a(table(), c);
  Engine b(table(), unscreened);
  std::vector<MeasurementResult> ra;
  std::vector<MeasurementResult> rb;
  std::vector<Measurement> scan;
  std::size_t offered = 0;
  for (std::size_t i = 0; i < items.size(); ++i) {
    scan.push_back(items[i].m);
    if (items[i].scan_break % 4 == 0 || i + 1 == items.size()) {
      a.on_scan(scan, ra);
      b.on_scan(scan, rb);
      offered += scan.size();
      scan.clear();
      check(ra.size() == offered);
      check_bounds(a);
      check_counts(a, offered);
    }
  }
  check(ra == rb);
  check(same_tracks(a, b));

  // ---- B: the pipeline, as the daemon feeds it --------------------------------
  std::string out_a;
  std::string out_b;
  net::Pipeline pa(table(), pc, [&](std::string_view l) { out_a.append(l); });
  net::PipelineConfig pc_unscreened = pc;
  pc_unscreened.engine = unscreened;
  net::Pipeline pb(table(), pc_unscreened, [&](std::string_view l) { out_b.append(l); });
  std::int64_t recv = 1'000'000'000;
  for (const Item& it : items) {
    recv += static_cast<std::int64_t>(it.recv_step) * 10'000'000;
    const std::vector<std::uint8_t> bytes = wire::encode(it.m);
    if ((it.scan_break & 0x40U) != 0) {
      pa.on_tick(Timestamp{recv});
      pb.on_tick(Timestamp{recv});
    }
    pa.on_frame(Timestamp{recv}, bytes);
    pb.on_frame(Timestamp{recv}, bytes);
    check(pa.held() <= pc.reorder_capacity);
    check(pa.timeline_held() <= 4 * pc.reorder_capacity);  // four candidates at most
    check_bounds(pa.engine());
    // Whenever a tick is asked for, taking it must settle that request: a run
    // that keeps asking would grow its log for ever.
    if (const auto due = pa.next_idle_tick(1'000'000'000); due && due->ns <= recv) {
      pa.on_tick(Timestamp{recv});
      pb.on_tick(Timestamp{recv});
      const auto again = pa.next_idle_tick(1'000'000'000);
      check(!again || again->ns > recv);
    }
  }
  pa.finish(Timestamp{recv});
  pb.finish(Timestamp{recv});
  check(!pa.has_pending());
  check(pa.timeline_held() == 0);
  check(out_a == out_b);
  check(same_tracks(pa.engine(), pb.engine()));
  // Frames that decode either reach the engine or are counted on the way.
  std::uint64_t decode_errors = 0;
  for (std::size_t e = 0; e < 9; ++e) {
    decode_errors += pa.decode_errors(static_cast<wire::DecodeError>(e));
  }
  check(pa.frames() == items.size());
  check(pa.engine().measurements_processed() + pa.duplicates() + pa.future_rejected() +
            pa.timeline_rejected() + decode_errors ==
        items.size());
  return 0;
}
