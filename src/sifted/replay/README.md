# replay

Append-only event log of everything the engine thread consumed (frames and scan-flush ticks, with receive times), and the replayer that feeds a log back through the same ingest path (`net::Pipeline`) using a `SimulatedClock`. Implemented in M2; format in ADR 0005.

- `event_log.hpp`: header text (format version, every run parameter, tangent-plane origin, canonical sensor table), binary records with CRC-32, strict `EventLogReader`, `EventLogWriter`.
- `replayer.hpp`: `replay(reader, pipeline)`.
- `crc32.hpp`: CRC-32 (IEEE).

Replaying a recorded run reproduces its output byte for byte (`tests/event_log_test.cpp`). Fuzz target: `fuzz/fuzz_event_log.cpp`.
