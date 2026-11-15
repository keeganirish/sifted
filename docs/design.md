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
| `core` | types, `Clock`, `Measurement` validation, `SensorTable`, `Engine` | clock, validation, sensor table done; engine is single-track (M1) |
| `wire` | versioned, size-limited, strict frame codec | **done** (v1, fuzzed) |
| `estimation` | Kalman/EKF per track | **done for M1**: CV Kalman filter + range-bearing EKF update |
| `association` | gating, assignment, track lifecycle | TODO (M1-2) |
| `store` | track table + decision trace | TODO (M2) |
| `replay` | event log, simulated-clock replayer | TODO (M2) |
| `net` | UDP ingest, WebSocket publisher, degraded-network sim | TODO (M2) |

## 5. Milestones

Scope is cut before polish. Each milestone ends with green CI and updated docs.

- **M0 (this skeleton):** build, CI, sanitizers, fuzz harness, wire codec, docs.
- **M1:** radar simulator with ground truth; single-sensor tracker (linear KF
  in a local Cartesian frame, EKF for range-bearing); sensor table (ADR 0003);
  first ground-truth regression scenarios (RMSE bound). *Implemented.* The
  camera (bearing-only) simulator moved to M2: one bearing-only sensor cannot
  observe range, so it only becomes useful once it is fused with radar.
- **M2:** multi-sensor association + track lifecycle; UDP ingest; record and
  replay; WebSocket stream and basic Leaflet map; ADS-B ingest (OpenSky).
- **M3:** degraded-network tests; broader fuzz and fault injection; benchmarks;
  finish design doc and threat model; demo video.
- **Stretch:** CUDA association, only after M3.

## 6. Estimation (M1)

**Frame.** Local flat x/y plane in metres, +x east, +y north; angles are
radians counter-clockwise from +x. A sensor's pose (position, boresight yaw)
comes from the sensor table (ADR 0003); measured bearings are relative to the
boresight.

**Filter.** `estimation::ConstantVelocityFilter`, state `[px, py, vx, vy]`,
continuous white-noise-acceleration process noise with PSD `q` per axis.
`predict(dt)` takes seconds explicitly; `update(z, R)` is the linear
position update. All updates go through one Joseph-form
`update_with_innovation(y, H, R)` with a Cholesky solve of `S`; `P` is
re-symmetrised after every step. Every operation validates its inputs and
either commits a finite result or leaves the state untouched.

**EKF.** `update_range_bearing()` evaluates `h(x) = [r, wrap(atan2 - yaw)]` and
its analytic Jacobian at the predicted state. The bearing innovation is
wrapped with `std::remainder(., 2*pi)` (exact, libm-independent), so a
measurement across the +/-pi seam is a small correction. Updates are refused
when the predicted range is under 1 m (the Jacobian is singular at 0).

**Engine (M1).** One track, no association. The first usable range-bearing
measurement initialises it (position from the inverse model with first-order
covariance, zero velocity with a 100 m/s prior). Each later measurement is
predicted to and applied on a copy of the filter, committed only if both steps
succeed. Measurements are rejected and counted, never applied, when they are
invalid, from an unknown sensor, of the wrong or an unsupported kind, a
duplicate of that sensor's last `seq`, or older than the track (drop +
counter, per the open question below). Gaps longer than one hour are refused
by the filter; track restart and coasting are M2 lifecycle work.

## 7. Tuning and baseline accuracy

`EngineConfig::accel_psd` defaults to `q = 4 m^2/s^3`, from the rule of thumb
`q ~ (0.5..1) * a_max^2 * T` for manoeuvres up to 2-3 m/s^2 at `T = 1 s`. It
trades straight-line smoothing against lag in turns; on the M1 scenarios:

| q [m^2/s^3] | straight_crossing RMSE / raw | turn RMSE / raw |
|---|---|---|
| 1 | 0.43 | 0.95 |
| 4 (default) | 0.50 | 0.72 |
| 16 | 0.59 | 0.73 |

The regression test asserts the default's values with
margin; see the header of `tests/scenario_regression_test.cpp`. Manoeuvre
handling beyond process noise (IMM, adaptive `q`) is out of scope for now.

## 8. Open questions

- Local tangent-plane origin for ADS-B (fixed per run vs. first fix). Leaning fixed per run, recorded in the log header.
- Out-of-order measurements: drop, or retrodict within a small window. Start with drop + counter. *(M1 engine drops and counts.)*
- WebSocket library vs. a minimal hand-rolled server (adds a dependency vs. attack surface).
