#include "sifted/replay/replayer.hpp"

namespace sifted::replay {

Result<ReplayStats, LogError> replay(EventLogReader& reader, net::Pipeline& pipeline,
                                     const std::function<void(Timestamp)>& before_record,
                                     const std::function<void()>& before_finish) {
  ReplayStats stats;
  SimulatedClock clock;
  bool started = false;
  while (true) {
    auto rec = reader.next();
    if (!rec.ok()) {
      return rec.error();
    }
    if (!rec.value()) {
      break;
    }
    const LogRecord& r = *rec.value();
    if (!started) {
      clock = SimulatedClock(r.recv);
      started = true;
    } else if (!clock.advance_to(r.recv)) {
      return LogError::TimeWentBackwards;  // the reader already refuses this; defence in depth
    }
    if (before_record) {
      before_record(clock.now());
    }
    if (r.type == RecordType::Frame) {
      ++stats.frames;
      pipeline.on_frame(clock.now(), r.payload);
    } else {
      ++stats.ticks;
      pipeline.on_tick(clock.now());
    }
  }
  stats.truncated = reader.torn_tail();
  stats.torn_bytes = reader.torn_bytes();
  if (before_finish) {
    before_finish();
  }
  pipeline.finish(clock.now());
  return stats;
}

}  // namespace sifted::replay
