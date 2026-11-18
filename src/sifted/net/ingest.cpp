#include "sifted/net/ingest.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "sifted/net/snapshot.hpp"

namespace sifted::net {
namespace {

// Bounds the guard's state however many different timelines a sender invents.
constexpr std::size_t kMaxTimelineCandidates = 4;

std::string token(std::string_view s) {
  std::string out(s);
  std::replace(out.begin(), out.end(), ' ', '_');
  return out;
}

double seconds_between(Timestamp from, Timestamp to) noexcept {
  const auto diff = static_cast<std::uint64_t>(to.ns) - static_cast<std::uint64_t>(from.ns);
  return static_cast<double>(diff) * 1e-9;
}

// to - from for from <= to, in uint64 so extreme timestamps cannot overflow.
std::uint64_t distance(Timestamp from, Timestamp to) noexcept {
  return static_cast<std::uint64_t>(to.ns) - static_cast<std::uint64_t>(from.ns);
}

std::uint64_t elapsed(Timestamp from, Timestamp to) noexcept {
  return to > from ? distance(from, to) : 0;
}

Timestamp saturating_add(Timestamp t, std::uint64_t ns) noexcept {
  const std::uint64_t room = distance(t, Timestamp{std::numeric_limits<std::int64_t>::max()});
  return ns >= room ? Timestamp{std::numeric_limits<std::int64_t>::max()}
                    : Timestamp{static_cast<std::int64_t>(static_cast<std::uint64_t>(t.ns) + ns)};
}

std::int64_t to_ms(std::uint64_t ns) noexcept { return static_cast<std::int64_t>(ns / 1'000'000); }

std::string frame_line(char kind, Timestamp recv, const Measurement& m) {
  return std::string(1, kind) + " " + std::to_string(recv.ns) + " " + std::to_string(m.sensor_id) +
         " " + std::to_string(m.seq) + " " + std::to_string(m.time.ns);
}

}  // namespace

bool is_valid(const PipelineConfig& c) noexcept {
  const bool guard_ok =
      c.max_time_jump_s == 0.0 ||
      (c.timeline_confirm_frames >= 2 && c.timeline_confirm_frames <= c.reorder_capacity &&
       c.timeline_confirm_s >= 0.0 && c.timeline_confirm_s <= 86400.0);
  return sifted::is_valid(c.engine) && c.snapshot_interval_s >= 0.0 &&
         std::isfinite(c.snapshot_interval_s) &&
         c.snapshot_trace_events <= kMaxSnapshotTraceEvents && c.max_scan_size >= 1 &&
         c.max_scan_size <= 4096 && c.dedup_window <= 65536 && c.reorder_window_s >= 0.0 &&
         c.reorder_window_s <= 60.0 && c.reorder_capacity >= 1 && c.reorder_capacity <= 65536 &&
         c.max_time_ahead_s >= 0.0 && c.max_time_ahead_s <= 86400.0 && c.max_time_jump_s >= 0.0 &&
         c.max_time_jump_s <= 86400.0 && guard_ok && c.feed_stale_factor >= 0.0 &&
         c.feed_stale_factor <= 1000.0 && c.feed_stale_min_s >= 0.0 &&
         c.feed_stale_min_s <= 86400.0;
}

Pipeline::Pipeline(SensorTable sensors, PipelineConfig config, LineSink output,
                   SnapshotSink publish)
    : config_(config),
      engine_(std::move(sensors), config.engine),
      dedup_(engine_.sensors(), config.dedup_window),
      output_(std::move(output)),
      publish_(std::move(publish)) {
  if (!is_valid(config_)) {
    throw std::invalid_argument("sifted::net::Pipeline: invalid PipelineConfig");
  }
  pending_.reserve(config_.max_scan_size);
  reorder_window_ns_ = std::llround(config_.reorder_window_s * 1e9);
  max_ahead_ns_ = std::llround(config_.max_time_ahead_s * 1e9);
  max_jump_ns_ = std::llround(config_.max_time_jump_s * 1e9);
  confirm_ns_ = std::llround(config_.timeline_confirm_s * 1e9);
  stale_min_ns_ = std::llround(config_.feed_stale_min_s * 1e9);
  if (config_.feed_stale_factor > 0.0) {
    activity_.resize(engine_.sensors().size());
  }
}

void Pipeline::emit(std::string line) {
  if (output_) {
    line += '\n';
    output_(line);
  }
}

// Compared as a difference in uint64 so extreme timestamps cannot overflow.
bool Pipeline::too_far_ahead(Timestamp recv, const Measurement& m) const noexcept {
  return max_ahead_ns_ > 0 && m.time > recv &&
         distance(recv, m.time) > static_cast<std::uint64_t>(max_ahead_ns_);
}

void Pipeline::on_frame(Timestamp recv, std::span<const std::uint8_t> frame) {
  ++frames_;
  const auto decoded = wire::decode_measurement(frame);
  if (!decoded.ok()) {
    ++decode_errors_[static_cast<std::size_t>(decoded.error())];
    emit("x " + std::to_string(recv.ns) + " " + token(wire::to_string(decoded.error())));
    return;
  }
  const Measurement& m = decoded.value();
  const bool gated = too_far_ahead(recv, m);
  if (max_jump_ns_ > 0 && !gated) {
    if (!engine_.can_accept(m)) {
      // It can never reach a track or the clock, so it says nothing about the
      // timeline and must not split a scan or sit in the reorder stage.
      results_.clear();
      engine_.on_scan(std::span<const Measurement>(&m, 1), results_);
      emit(frame_line('m', recv, m) + " " + token(to_string(results_[0].outcome)) + " " +
           std::to_string(results_[0].track));
      return;
    }
    if (!passes_timeline_guard(recv, m)) {
      return;
    }
  }
  if (dedup_.is_duplicate(m.sensor_id, m.seq, m.time)) {
    emit(frame_line('d', recv, m));
    return;
  }
  if (gated) {
    ++future_rejected_;
    emit(frame_line('f', recv, m));
    return;
  }
  admit(recv, m);
}

void Pipeline::admit(Timestamp recv, const Measurement& m) {
  note_activity(recv, m);
  if (reorder_window_ns_ == 0) {
    assemble(recv, m);
    return;
  }
  if (released_ && m.time < *released_) {
    ++reorder_late_;  // its slot has already gone by: holding it cannot help
    assemble(recv, m);
    release(recv, false);
    return;
  }
  const Held h{m, recv, arrivals_++};
  const auto pos =
      std::upper_bound(held_.begin(), held_.end(), h, [](const Held& a, const Held& b) {
        if (a.m.time != b.m.time) {
          return a.m.time < b.m.time;
        }
        if (a.m.sensor_id != b.m.sensor_id) {
          return a.m.sensor_id < b.m.sensor_id;
        }
        return a.order < b.order;
      });
  held_.insert(pos, h);
  held_high_water_ = std::max(held_high_water_, held_.size());
  release(recv, false);
}

// ---- timeline guard (ADR 0012) ----------------------------------------------

bool Pipeline::on_timeline(Timestamp head, Timestamp head_recv, Timestamp t,
                           Timestamp recv) const noexcept {
  const auto jump = static_cast<std::uint64_t>(max_jump_ns_);
  if (t <= head) {
    return distance(t, head) <= jump;
  }
  const std::uint64_t waited = elapsed(head_recv, recv);
  const std::uint64_t allowed = waited > std::numeric_limits<std::uint64_t>::max() - jump
                                    ? std::numeric_limits<std::uint64_t>::max()
                                    : waited + jump;
  return distance(head, t) <= allowed;
}

bool Pipeline::passes_timeline_guard(Timestamp recv, const Measurement& m) {
  if (timeline_head_ && on_timeline(*timeline_head_, timeline_head_recv_, m.time, recv)) {
    if (m.time > *timeline_head_) {
      timeline_head_ = m.time;
      timeline_head_recv_ = recv;
    }
    timeline_support_recv_ = recv;
    discard_candidates(recv);  // the current timeline is alive: nothing replaces it
    return true;
  }
  quarantine(recv, m);
  settle_timeline(recv);
  return false;
}

void Pipeline::quarantine(Timestamp recv, const Measurement& m) {
  for (Candidate& c : candidates_) {
    if (!on_timeline(c.head, c.head_recv, m.time, recv)) {
      continue;
    }
    // A repeated datagram is not a second witness.
    if (c.frames.size() >= config_.reorder_capacity ||
        !c.keys.emplace(m.sensor_id, m.seq, m.time.ns).second) {
      reject(recv, m);
      return;
    }
    c.frames.push_back({m, recv, arrivals_++});
    if (m.time > c.head) {
      c.head = m.time;
      c.head_recv = recv;
    }
    return;
  }
  if (candidates_.size() >= kMaxTimelineCandidates) {
    // The timeline with the least evidence makes way (the oldest of equals).
    const auto weakest = std::min_element(
        candidates_.begin(), candidates_.end(),
        [](const Candidate& a, const Candidate& b) { return a.frames.size() < b.frames.size(); });
    for (const Held& h : weakest->frames) {
      reject(recv, h.m);
    }
    candidates_.erase(weakest);
  }
  Candidate c{m.time, recv, recv, {}, {}};
  c.frames.push_back({m, recv, arrivals_++});
  c.keys.emplace(m.sensor_id, m.seq, m.time.ns);
  candidates_.push_back(std::move(c));
}

// The earliest receive time at which `c` can be adopted, if evidence is all it
// still lacks. At start-up there is nothing to protect: enough frames are
// adopted at once, and a lone frame after the confirmation time; a wrong choice
// is corrected like any other timeline change.
std::optional<Timestamp> Pipeline::adoption_time(const Candidate& c) const noexcept {
  const auto confirm = static_cast<std::uint64_t>(confirm_ns_);
  const bool enough = c.frames.size() >= config_.timeline_confirm_frames;
  if (!timeline_head_) {
    return enough ? c.first_recv : saturating_add(c.first_recv, confirm);
  }
  if (!enough) {
    return std::nullopt;
  }
  return saturating_add(std::max(c.first_recv, timeline_support_recv_), confirm);
}

void Pipeline::settle_timeline(Timestamp recv) {
  std::optional<std::size_t> best;
  for (std::size_t i = 0; i < candidates_.size(); ++i) {
    const auto at = adoption_time(candidates_[i]);
    if (at && recv >= *at &&
        (!best || candidates_[i].frames.size() > candidates_[*best].frames.size())) {
      best = i;
    }
  }
  if (best) {
    adopt(*best, recv);
  }
}

void Pipeline::adopt(std::size_t candidate, Timestamp recv) {
  Candidate c = std::move(candidates_[candidate]);
  candidates_.erase(candidates_.begin() + static_cast<std::ptrdiff_t>(candidate));
  discard_candidates(recv);
  // Whatever is left of the old timeline goes first.
  release(recv, true);
  flush(recv);

  std::sort(c.frames.begin(), c.frames.end(), [](const Held& a, const Held& b) {
    return std::tie(a.m.time, a.m.sensor_id, a.order) < std::tie(b.m.time, b.m.sensor_id, b.order);
  });
  const bool restart = engine_.time() && c.frames.front().m.time < *engine_.time();
  if (restart) {
    engine_.restart_timeline(c.frames.front().m.time);
    last_snapshot_.reset();
    released_.reset();
    ++timeline_restarts_;
  }
  if (timeline_head_) {
    ++timeline_switches_;
  }
  timeline_head_ = c.head;
  timeline_head_recv_ = c.head_recv;
  timeline_support_recv_ = recv;
  emit("T " + std::to_string(recv.ns) + " " + std::to_string(c.head.ns) + " " +
       std::to_string(c.frames.size()) + (restart ? " 1" : " 0"));
  for (const Held& h : c.frames) {
    if (dedup_.is_duplicate(h.m.sensor_id, h.m.seq, h.m.time)) {
      emit(frame_line('d', recv, h.m));
      continue;
    }
    note_activity(h.recv, h.m);
    released_ = h.m.time;  // held for longer than any reorder window already
    assemble(recv, h.m);
  }
}

void Pipeline::reject(Timestamp recv, const Measurement& m) {
  ++timeline_rejected_;
  emit(frame_line('t', recv, m));
}

void Pipeline::discard_candidates(Timestamp recv) {
  for (const Candidate& c : candidates_) {
    for (const Held& h : c.frames) {
      reject(recv, h.m);
    }
  }
  candidates_.clear();
}

std::size_t Pipeline::timeline_held() const noexcept {
  std::size_t n = 0;
  for (const Candidate& c : candidates_) {
    n += c.frames.size();
  }
  return n;
}

// ---- feed status (ADR 0012) ---------------------------------------------------

void Pipeline::note_activity(Timestamp recv, const Measurement& m) {
  if (activity_.empty()) {
    return;
  }
  const auto index = engine_.sensors().index_of(m.sensor_id);
  if (!index) {
    return;
  }
  SensorActivity& a = activity_[*index];
  if (!a.last_time) {
    a.scan_recv = recv;
  } else if (*a.last_time != m.time) {  // frames of one scan say nothing about the cadence
    a.gaps[a.gap_next] = static_cast<std::int64_t>(std::min<std::uint64_t>(
        elapsed(a.scan_recv, recv),
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())));
    a.gap_next = (a.gap_next + 1) % a.gaps.size();
    a.gap_count = std::min(a.gap_count + 1, a.gaps.size());
    a.scan_recv = recv;
  }
  a.last_time = m.time;
  if (!a.last_recv || recv > *a.last_recv) {
    a.last_recv = recv;
  }
}

std::int64_t Pipeline::stale_after_ns(const SensorActivity& a) const noexcept {
  if (a.gap_count == 0) {
    return stale_min_ns_;
  }
  std::array<std::int64_t, 5> gaps = a.gaps;
  const auto mid = gaps.begin() + static_cast<std::ptrdiff_t>(a.gap_count / 2);
  std::nth_element(gaps.begin(), mid, gaps.begin() + static_cast<std::ptrdiff_t>(a.gap_count));
  const double scaled = config_.feed_stale_factor * static_cast<double>(*mid);
  const auto limit = static_cast<double>(std::numeric_limits<std::int64_t>::max() / 2);
  return std::max(stale_min_ns_, static_cast<std::int64_t>(std::min(scaled, limit)));
}

FeedStatus Pipeline::feed_status(Timestamp recv) {
  FeedStatus f;
  const auto& table = engine_.sensors().sensors();
  f.sensors.reserve(table.size());
  std::size_t live = 0;
  std::size_t stale = 0;
  for (std::size_t i = 0; i < table.size(); ++i) {
    SensorActivity& a = activity_[i];
    SensorFeed s;
    s.id = table[i].id;
    const std::int64_t after = stale_after_ns(a);
    s.stale_after_ms = to_ms(static_cast<std::uint64_t>(after));
    if (a.last_recv) {
      const std::uint64_t age = elapsed(*a.last_recv, recv);
      a.stale = age >= static_cast<std::uint64_t>(after);
      s.state = a.stale ? FeedState::Stale : FeedState::Live;
      s.age_ms = to_ms(age);
      f.age_ms = f.age_ms ? std::min(*f.age_ms, *s.age_ms) : *s.age_ms;
      (a.stale ? stale : live) += 1;
    }
    f.sensors.push_back(s);
  }
  f.state = live + stale == 0 ? FeedState::Waiting
            : stale == 0      ? FeedState::Live
            : live == 0       ? FeedState::Stale
                              : FeedState::Degraded;
  return f;
}

void Pipeline::publish_snapshot(Timestamp time, const std::vector<TrackEstimate>& tracks,
                                const std::optional<FeedStatus>& feed) {
  const std::string json =
      format_snapshot(++snapshot_seq_, time, tracks, config_.engine.origin, feed);
  emit("s " + json);
  if (publish_) {
    publish_(json);
  }
}

// A tick with nothing else to do: tell the operator what the silence means.
void Pipeline::publish_silence(Timestamp recv) {
  const auto now = engine_.time();
  if (activity_.empty() || !now) {
    return;
  }
  FeedStatus feed = feed_status(recv);
  // Data that did not move the clock (late frames, say) still ends the silence.
  predicting_ = predicting_ && feed.state == FeedState::Stale;
  const bool changed = feed_shown_ != feed.state;
  const bool moving = feed.state == FeedState::Stale && predicting_;
  if (!changed && !moving) {
    return;
  }
  feed_shown_ = feed.state;
  if (feed.state != FeedState::Stale) {
    publish_snapshot(*now, engine_.tracks_at(*now, config_.snapshot_trace_events), feed);
    return;
  }
  // Counted from the newest frame received, not from when the reorder stage let
  // it through: the picture is as old as its data.
  std::optional<Timestamp> newest;
  for (const SensorActivity& a : activity_) {
    if (a.last_recv && (!newest || *a.last_recv > *newest)) {
      newest = a.last_recv;
    }
  }
  const Timestamp at = saturating_add(*now, newest ? elapsed(*newest, recv) : 0);
  feed.predicted_ms = to_ms(distance(*now, at));
  const auto tracks = engine_.tracks_if_silent_until(at, config_.snapshot_trace_events);
  predicting_ = !tracks.empty();
  silence_published_ = recv;
  publish_snapshot(at, tracks, feed);
}

std::optional<Timestamp> Pipeline::next_idle_tick(std::int64_t period_ns) const {
  std::optional<Timestamp> due;
  const auto consider = [&due](Timestamp t) {
    if (!due || t < *due) {
      due = t;
    }
  };
  for (const Candidate& c : candidates_) {
    if (const auto at = adoption_time(c)) {
      consider(*at);
    }
  }
  if (activity_.empty() || !engine_.time()) {
    return due;
  }
  for (const SensorActivity& a : activity_) {
    if (a.last_recv && !a.stale) {
      consider(saturating_add(*a.last_recv, static_cast<std::uint64_t>(stale_after_ns(a))));
    }
  }
  if (predicting_ && silence_published_) {
    consider(saturating_add(*silence_published_, static_cast<std::uint64_t>(period_ns)));
  }
  return due;
}

void Pipeline::assemble(Timestamp recv, const Measurement& m) {
  if (!pending_.empty() &&
      (pending_.front().sensor_id != m.sensor_id || pending_.front().time != m.time ||
       pending_.size() >= config_.max_scan_size)) {
    flush(recv);
  }
  pending_.push_back(m);
}

void Pipeline::release(Timestamp now, bool all) {
  if (held_.empty()) {
    return;
  }
  std::optional<Timestamp> watermark;
  if (all) {
    watermark = held_.back().m.time;
  } else {
    for (const Held& h : held_) {
      // Expired: received at least the window ago (h.recv <= now - window).
      const bool expired = now >= h.recv && static_cast<std::uint64_t>(now.ns) -
                                                    static_cast<std::uint64_t>(h.recv.ns) >=
                                                static_cast<std::uint64_t>(reorder_window_ns_);
      if (expired && (!watermark || h.m.time > *watermark)) {
        watermark = h.m.time;
      }
    }
  }
  std::size_t n = 0;
  if (watermark) {
    while (n < held_.size() && held_[n].m.time <= *watermark) {
      ++n;
    }
  }
  if (held_.size() - n > config_.reorder_capacity) {
    const std::size_t forced = held_.size() - n - config_.reorder_capacity;
    reorder_forced_ += forced;
    n += forced;
  }
  for (std::size_t i = 0; i < n; ++i) {
    released_ = held_[i].m.time;
    assemble(now, held_[i].m);
  }
  held_.erase(held_.begin(), held_.begin() + static_cast<std::ptrdiff_t>(n));
}

void Pipeline::on_tick(Timestamp recv) {
  settle_timeline(recv);
  release(recv, false);
  flush(recv);
  publish_silence(recv);
}

void Pipeline::finish(Timestamp recv) {
  // A run too short to confirm its only timeline still has an answer.
  if (!timeline_head_ && !candidates_.empty()) {
    std::size_t best = 0;
    for (std::size_t i = 1; i < candidates_.size(); ++i) {
      if (candidates_[i].frames.size() > candidates_[best].frames.size()) {
        best = i;
      }
    }
    adopt(best, recv);
  }
  discard_candidates(recv);
  release(recv, true);
  flush(recv);
}

void Pipeline::flush(Timestamp recv) {
  if (pending_.empty()) {
    return;
  }
  ++scans_;
  results_.clear();
  const auto clock_before = engine_.time();
  engine_.on_scan(pending_, results_);
  if (engine_.time() != clock_before) {
    predicting_ = false;  // real data again: the next silence starts afresh
  }
  for (std::size_t i = 0; i < pending_.size(); ++i) {
    const Measurement& m = pending_[i];
    emit("m " + std::to_string(recv.ns) + " " + std::to_string(m.sensor_id) + " " +
         std::to_string(m.seq) + " " + std::to_string(m.time.ns) + " " +
         token(to_string(results_[i].outcome)) + " " + std::to_string(results_[i].track));
  }
  pending_.clear();

  const auto now = engine_.time();
  if (now &&
      (!last_snapshot_ || seconds_between(*last_snapshot_, *now) >= config_.snapshot_interval_s)) {
    last_snapshot_ = now;
    std::optional<FeedStatus> feed;
    if (!activity_.empty()) {
      feed = feed_status(recv);
      feed_shown_ = feed->state;
    }
    publish_snapshot(*now, engine_.tracks_at(*now, config_.snapshot_trace_events), feed);
  }
}

}  // namespace sifted::net
