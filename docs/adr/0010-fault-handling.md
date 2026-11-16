# ADR 0010: Fault handling -- clock gate, per-source rate limiting, and what is deliberately not mitigated

Status: accepted (M3)

## Context
M3 fault injection (`tests/fault_injection_test.cpp`, `tests/udp_flood_test.cpp`,
`sim/sifted_sim/faults.py`) measured how the system responds to sensor
restarts, dropouts, clock faults, biased and stuck sensors, duplicate floods,
malformed bursts and clutter floods. Most were already handled; two needed a
mechanism. Measured numbers are in docs/threat-model.md ("Sensor and input
faults").

## Decision
1. **Clock gate** (`PipelineConfig::max_time_ahead_s`, recorded in the log
   header). The engine trusts sensor time and drops measurements older than
   its clock. A sensor whose clock runs *ahead* therefore moves the engine
   clock forward and turns every other sensor's on-time data into
   OutOfOrder drops: measured, a camera 2 s fast made the radar lose
   223-379 measurements per run and 100% of target-instants were missed. The
   gate compares each frame's measurement time with its **receive time** --
   an input that the event log records, so replay stays exact and the core
   still reads no clock -- and rejects frames more than `max_time_ahead_s`
   ahead (output line `f`, counter). With the gate, exactly the bad sensor's
   frames are refused and the rest is processed normally. Leads smaller than
   the gate are absorbed by the reorder window (ADR 0009) as long as the gate
   is no wider than the window, so the daemon documents the pair together.
   **Off by default** (0): it needs senders and receiver on a common timebase
   (wall-clock senders such as ADS-B, or `send.py --rebase-now`); the
   simulated scenarios' timestamps are not wall-clock. A sensor that runs
   *behind* only loses its own data (OutOfOrder); no mitigation needed.
   Clock drift is the same problem growing over time: 5000 ppm costs radar
   scans after ~50 s unmitigated, nothing with window + gate.
2. **Per-source rate limiting** at the receive thread
   (`net::SourceRateLimiter`, a token bucket per source *address*; daemon
   defaults 20 000 datagrams/s, burst 4 096, 1 024 sources tracked, LRU
   eviction). Implements the threat model's open "per-source rate limiting"
   item. Drops are counted (`rate_limited`) and never reach the queue or the
   log. Tested on real sockets: a 20 000-datagram duplicate flood from one
   address does not cost a second, honest address a single datagram.
   Defaults: well above one sender's legitimate peak (a 4 096-detection burst
   covers max_scan_size scans from many sensors behind one address), well
   below what fills the 4 096-frame queue faster than the engine drains it
   for a typical picture (docs/benchmarks.md).

Deliberately not mitigated (documented, tested for bounded behaviour):
- **Biased sensors.** A constant bias is a registration problem; one sensor's
  data cannot reveal it. Alone, a 50 m range bias shifts the picture by the
  bias with identity intact; fused with an unbiased camera the two disagree
  and identity can break (measured 1-6 ID switches). Registration / bias
  estimation is future work.
- **Stuck (frozen) sensors.** Repeated scans look like stationary targets
  and are confirmed as such (at most one false track per frozen detection);
  real tracks coast and recover within ~2 s of the sensor recovering. A
  "frozen output" detector would need a notion of expected change per
  sensor; future work.
- **Spoofed source addresses** defeat per-address limits (UDP is
  unauthenticated); the bounded queue remains the backstop.

## Consequences
+ One bad clock can no longer silence the other sensors (when the gate is
  configured).
+ One flooding host cannot starve the others.
- The gate trades availability for integrity: a fast clock on the only
  track-starting sensor leaves no tracks at all rather than wrong ones.
