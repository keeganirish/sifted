# store

The track table and the trace of decisions behind each track: what answers "why does the system believe this?". Implemented in M2.

- `track_store.hpp`: `TrackRecord` (filter, status, timing, contributing sensors, birth and confirmation events, a bounded window of recent decisions, per-kind totals), `TrackStore` (live tracks sorted by id, bounded history of dropped tracks, bounded per-measurement outcome log) and `explain(id)`.
- `ring.hpp`: fixed-capacity, insertion-ordered ring buffer used for every bounded log.

Trace events: initiated, associated (with `d^2` and gate), competed (inside the gate but assigned elsewhere), near miss (outside the gate, within 4x), update rejected, confirmed, coasting, recovered, manoeuvre start/end, dropped (with reason). All bounds are in `TraceConfig` and recorded in event logs.
