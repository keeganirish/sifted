# net

Ingest and publication (ADR 0006). Network threads only receive, validate, timestamp and enqueue; they never touch the Engine. Implemented in M2; reorder stage, clock gate, rate limiting and the degraded-network model added in M3.

Deterministic part (in `sifted_core`, shared by live runs and replay):
- `ingest.hpp`: `Pipeline`: strict decode, duplicate suppression, clock gate (ADR 0010), bounded reorder stage (ADR 0009), scan assembly, `Engine::on_scan`, output lines and snapshots.
- `dedup.hpp`: per-sensor `(seq, time)` window, restart-tolerant.
- `durable_file.hpp`: the stream buffer behind a recording: bounded loss on a crash or power cut (ADR 0012). OS-facing, in `sifted_net`.
- `snapshot.hpp`: versioned track snapshot JSON (`sifted.tracks` v2), including each track's newest decision-trace events.
- `pipeline_config.hpp`: everything besides the sensor table that determines output (all of it recorded in the event log header).
- `impairment.hpp`: the seeded degraded-network model (loss with Gilbert-Elliott bursts, delay and jitter, duplication, outages; ADR 0008). Used by tests offline; mirrored exactly by `sim/sifted_sim/impair.py`, which also runs it live as a UDP proxy.

OS-facing edge (in `sifted_net`, linked only by the daemon, tests and fuzzers):
- `udp.hpp`: UDP receive thread (oversize, rate-limited and invalid datagrams dropped before queueing; every datagram counted exactly once).
- `rate_limit.hpp`: per-source-address token bucket with a bounded LRU table (ADR 0010).
- `bounded_queue.hpp`: bounded MPSC queue with drop counting.
- `system_clock.hpp`: wall clock for the live edge only, never backwards.
- `websocket.hpp`, `sha1.hpp`: minimal read-only RFC 6455 publisher; `parse_handshake` and `parse_client_frame` are pure and fuzzed (`fuzz/fuzz_websocket.cpp`).
