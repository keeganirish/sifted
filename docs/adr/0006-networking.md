# ADR 0006: POSIX UDP ingest, (seq, time) duplicate suppression, minimal read-only WebSocket server

Status: accepted (M2; amended M3). *Recorded retrospectively in M3 from
the M2 implementation, which referenced it.*

## Context
Sensors send small datagrams; operators need a live stream in a browser.
Both boundaries are untrusted (threat model B1, B3). Dependencies add attack
surface and packaging work.

## Decision
- **UDP ingest** with plain POSIX sockets on one receive thread: receive,
  drop oversize datagrams (buffer one byte larger than any valid frame),
  strict wire decode, stamp with the receive time, try to enqueue into a
  bounded queue (drop and count when full). The thread never touches the
  engine. **(M3)** A per-source-address token bucket runs before decoding
  (ADR 0010).
- **Duplicate suppression** in the deterministic pipeline: a frame is a
  duplicate if its `(seq, time)` matches one of the last `dedup_window`
  (256) keys accepted from that sensor. Keying on time as well as seq makes
  sensor restarts (seq reset) transparent -- tested in M3
  (`Faults.SensorRestartIsTransparent`). State is kept only for configured
  sensors, so spoofed ids cannot grow memory.
- **WebSocket**: a minimal RFC 6455 server, hand-rolled (handshake with
  SHA-1/base64, unmasked text frames out), read-only: from clients it accepts
  only masked, final Close/Ping/Pong frames of at most 125 bytes; anything
  else is refused with close code 1008. Slow clients skip snapshots instead
  of queueing them; stalled clients are dropped. **(M3)** The client-frame
  parser is a pure function (`parse_client_frame`) so it can be fuzzed.

## Alternatives considered
- A WebSocket library (e.g. uWebSockets, Boost.Beast): more features we do
  not need, a large dependency, more attack surface. Resolves design.md open
  question 3 in favour of the minimal server.
- TCP for sensors: head-of-line blocking and connection state per sensor for
  no benefit; loss is handled by the tracker.

## Consequences
+ No networking dependency; every parser of client input is small and fuzzed.
- No TLS, no authentication (accepted; threat model).
