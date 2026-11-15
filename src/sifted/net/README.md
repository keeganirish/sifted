# net

Ingest and publication (ADR 0006). Network threads only receive, validate, timestamp and enqueue; they never touch the Engine. Implemented in M2.

Deterministic part (in `sifted_core`, shared by live runs and replay):
- `ingest.hpp`: `Pipeline`: strict decode, duplicate suppression, scan assembly, `Engine::on_scan`, output lines and snapshots.
- `dedup.hpp`: per-sensor `(seq, time)` window, restart-tolerant.
- `snapshot.hpp`: versioned track snapshot JSON (`sifted.tracks` v1).
- `pipeline_config.hpp`: everything besides the sensor table that determines output.

OS-facing edge (in `sifted_net`, linked only by the daemon and tests):
- `udp.hpp`: UDP receive thread (oversize and invalid datagrams dropped before queueing).
- `bounded_queue.hpp`: bounded MPSC queue with drop counting.
- `system_clock.hpp`: wall clock for the live edge only, never backwards.
- `websocket.hpp`, `sha1.hpp`: minimal read-only RFC 6455 publisher.

Still to come (M3): the degraded-network simulator (loss, delay, reorder, link drop).
