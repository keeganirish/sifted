#pragma once

#include <functional>
#include <istream>
#include <optional>

#include "sifted/core/clock.hpp"
#include "sifted/core/result.hpp"
#include "sifted/net/ingest.hpp"
#include "sifted/replay/event_log.hpp"

namespace sifted::replay {

struct ReplayStats {
  std::uint64_t frames = 0;
  std::uint64_t ticks = 0;
};

// Feeds every record of `reader` through `pipeline` (which must have been
// built from reader.header()) in log order, driving a SimulatedClock from the
// recorded receive times: the same ingest path, with time as input. Calls
// pipeline.finish() at the end of the log, as the live daemon does at
// shutdown. `before_record`, if set, is called with each record's receive time
// before it is processed (e.g. to pace a replay for display); it must not
// affect the pipeline.
Result<ReplayStats, LogError> replay(EventLogReader& reader, net::Pipeline& pipeline,
                                     const std::function<void(Timestamp)>& before_record = {});

}  // namespace sifted::replay
