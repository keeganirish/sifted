# fuzz

libFuzzer targets (build with the `fuzz` preset: Clang + ASan + UBSan + libFuzzer).
Every parser of untrusted or semi-trusted input has one, and the engine has a
structure-aware one. Each file's header lists the properties it checks.

| Target | Input | Seed corpus |
|---|---|---|
| `fuzz_decode` | wire frames (B1): decode, canonical re-encode | `corpus/` |
| `fuzz_event_log` | whole event logs (B4): open, header round trip, replay through the pipeline | `corpus_log/` |
| `fuzz_log_header` | event log header text (B4), without the CRC that hides it from `fuzz_event_log` | `corpus_header/` |
| `fuzz_sensor_table` | sensor table text (B5): parse, canonical round trip | `corpus_sensors/` |
| `fuzz_websocket` | WebSocket handshake and client frame stream (B3) | `corpus_ws/` |
| `fuzz_track_report` | track reports (B7, tracker output read by the evaluation): parse, canonical round trip, scoring against a fixed truth never fails other than with `TrackReportError` and gives a finite GOSPA | `corpus_tracks/` |
| `fuzz_engine` | structure-aware measurement sequences into two engines and two pipelines: one outcome per measurement, track cap, every bounded buffer bounded, finite state, determinism (the second engine runs without the gate screen) | `corpus_engine/` |

```bash
cmake --preset fuzz && cmake --build --preset fuzz
mkdir -p build/fuzz-work/engine
build/fuzz/fuzz/fuzz_engine build/fuzz-work/engine fuzz/corpus_engine -max_total_time=60
```

libFuzzer writes new inputs into the FIRST corpus directory, so always give a
scratch directory first and copy only interesting inputs into `fuzz/` by hand
(`found_*` files were added that way, after `-merge=1`). The CI fuzz-smoke job
runs every target for 60 s. The Python property tests for the OpenSky adapter
are in `sim/tests/test_adsb_fuzz.py`.
