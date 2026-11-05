# Sifted design

> Status: skeleton. Sections marked **TODO** are filled in as the milestone that
> builds them lands. Decisions are recorded in [`adr/`](adr/).

## 1. Goals and non-goals

**Goals:** deterministic, traceable, robust-to-untrusted-input multi-sensor
track fusion, with measured performance.
**Non-goals:** anything involving weapons or targeting; non-public data;
production-grade security hardening or a polished product UI.

## 2. Architecture

```
sensors --UDP--> [net: receive + parse + validate] --queue--> [Engine: associate -> estimate -> store]
                         |                                              |
                         +--> event log (raw frames + receive time)     +--> WebSocket publisher --> operator map
```

Threading model: network threads only receive, validate, timestamp and enqueue.
**One** engine thread consumes the queue in order. Publisher threads read
immutable track snapshots. No shared mutable state is touched from two threads
without a documented owner (checked by TSan in CI).

## 3. Determinism contract

Replaying a recorded log must reproduce the original output **bit for bit**.
Rules (enforced by review, build flags and tests):

1. Time is an input. Core code takes a `Clock`/`Timestamp`; only the live app
   edge reads the machine clock, and it records what it read.
2. The engine is single-threaded and a pure function of its ordered input.
3. No `-ffast-math`; `-ffp-contract=off` so FMA contraction cannot differ
   between builds (see `cmake/SiftedOptions.cmake`).
4. No iteration over address- or hash-ordered containers where order affects
   output; ties are broken by id.
5. Timestamps and ids are integers.

Known limit: results are guaranteed identical for the same binary and
architecture. Cross-compiler/cross-libm bit-equality of transcendental
functions (`sin`, `atan2`) is **not** promised; CI replays on the same build.

## 4. Modules

| Dir | Responsibility | Status |
|---|---|---|
| `core` | types, `Clock`, `Measurement` validation, `Engine` | clock + validation done; engine is a placeholder |
| `wire` | versioned, size-limited, strict frame codec | **done** (v1, fuzzed) |
| `estimation` | Kalman/EKF per track | TODO (M1) |
| `association` | gating, assignment, track lifecycle | TODO (M1-2) |
| `store` | track table + decision trace | TODO (M2) |
| `replay` | event log, simulated-clock replayer | TODO (M2) |
| `net` | UDP ingest, WebSocket publisher, degraded-network sim | TODO (M2) |

## 5. Milestones

Scope is cut before polish. Each milestone ends with green CI and updated docs.

- **M0 (this skeleton):** build, CI, sanitizers, fuzz harness, wire codec, docs.
- **M1:** simulator (radar + camera) with ground truth; single-sensor tracker
  (linear KF in a local Cartesian frame, EKF for range-bearing); first
  ground-truth regression scenario (RMSE bound).
- **M2:** multi-sensor association + track lifecycle; UDP ingest; record and
  replay; WebSocket stream and basic Leaflet map; ADS-B ingest (OpenSky).
- **M3:** degraded-network tests; broader fuzz and fault injection; benchmarks;
  finish design doc and threat model; demo video.
- **Stretch:** CUDA association, only after M3.

## 6. Open questions

- Local tangent-plane origin for ADS-B (fixed per run vs. first fix). Leaning fixed per run, recorded in the log header.
- Out-of-order measurements: drop, or retrodict within a small window. Start with drop + counter.
- WebSocket library vs. a minimal hand-rolled server (adds a dependency vs. attack surface).
