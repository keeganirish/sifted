# ADR 0005: Append-only event log of engine-thread input, replayed through the same pipeline

Status: accepted (M2; amended M3). *Recorded retrospectively in M3 from
the M2 implementation, which referenced it. `src/sifted/replay/event_log.hpp`
holds the normative format description.*

## Context
Replay must reproduce a live run's output bit for bit (ADR 0002). The live
run has non-deterministic inputs: datagram arrival times and order, and the
moments the engine thread decides the input went quiet.

## Decision
Record **what the engine thread consumed, in the order it consumed it**:
every frame with its receive time, and every scan-flush "tick" with its time.
Replay feeds the records through the same `net::Pipeline` with a
`SimulatedClock`, so everything downstream (decoding, duplicate suppression,
reordering, scan assembly, the engine, snapshots) is recomputed, not trusted.

```
file    := preamble header_text header_crc record*
preamble:= "SIFTLOG\0" | version u16 (=1) | reserved u16 | header_len u32 (<= 128 KiB)
record  := type u8 (1 frame, 2 tick) | reserved u8 | len u16 | recv_ns i64 | payload | crc u32
```

The header text records every parameter that affects output ("param key
value" lines in shortest round-trip form), the tangent-plane origin and the
canonical sensor table, so a log replays on its own. Receive times never
decrease. The reader checks sizes before allocating, CRCs, reserved fields,
types and time order (threat model B4); frames still go through the strict
wire decoder on replay.

**M3 amendment.** New parameters (`assignment`, `reorder_window_s`,
`reorder_capacity`, `max_time_ahead_s`) are written by every writer and are
*optional* for readers: a header without them reads with defaults that
reproduce the M2 behaviour exactly, so M2 recordings still replay. Unknown
keys and bad values are still refused. The format version stays 1.

**Amendment (ADR 0012).** A tick now records any moment the engine thread acted
on the passage of time, including idle duties such as reporting silence. New
optional parameters: `max_time_jump_s`, `timeline_confirm_frames`,
`timeline_confirm_s`, `feed_stale_factor`, `feed_stale_min_s`; a header without
them keeps the guard and the feed status off, so older logs replay byte for byte.
The daemon hands records to the OS on a bounded schedule, and a log whose final
record was torn by a crash can be replayed with an explicit recovery mode. The
format and the strict reader are unchanged.

## Consequences
+ Replay equality is a byte comparison of the output stream; tested for
  clean, impaired (loss, jitter, duplicates, outage) and faulted runs.
+ Anything the receive thread drops before the queue (oversize, invalid,
  rate-limited, queue-full) never reaches the log, so it cannot affect replay.
- Results are guaranteed identical for the same binary and architecture only
  (design.md section 3).
