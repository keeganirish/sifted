# Sifted design

> Status: complete through M3. Decisions are recorded in [`adr/`](adr/)
> (0001-0011). Measured numbers come from the tests and benchmarks named next
> to them; all were run with GCC 13 against Eigen 3.4.0 and GoogleTest 1.14.

## 1. Goals and non-goals

**Goals:** deterministic, traceable, robust-to-untrusted-input multi-sensor
track fusion, with measured performance.
**Non-goals:** anything involving weapons or targeting; non-public data;
production-grade security hardening or a polished product UI.

## 2. Architecture and threading

```
sensors --UDP--> [rx thread: size check -> per-source rate limit -> strict decode -> stamp recv time]
                       |  bounded queue (drop + count when full)
                       v
          [engine thread: net::Pipeline]
             decode -> dedup (seq, time) -> clock gate -> reorder stage -> scan assembly
             -> Engine (associate -> estimate -> store) -> output lines, snapshots
                       |                                        |
                       +--> event log (every frame and tick,    +--> WebSocket thread --> operator map
                            with its receive time)                   (latest snapshot only)
```

Threads and ownership (checked by TSan on the whole test suite, including
the live UDP flood tests with a concurrent consumer):

| Thread | Owns | Shares |
|---|---|---|
| UDP receive (`UdpReceiver::run`) | its socket, the rate limiter table | pushes into `BoundedQueue` (mutex); increments atomic `UdpStats` |
| engine (daemon main thread) | `Pipeline`, `Engine`, event log writer, output file | pops the queue; calls `WebSocketServer::publish` (swaps a shared pointer under a mutex) |
| WebSocket server | all client sockets | reads the latest snapshot pointer under the mutex; atomic stats |

The engine thread is the only one that touches tracker state. The receive
thread reads the wall clock (receive stamps) and the engine thread reads it
only to stamp quiet ticks; both values go into the event log, so replay never
reads a clock.

## 3. Determinism contract

Replaying a recorded log must reproduce the original output **bit for bit**.
Rules (enforced by review, build flags and tests):

1. Time is an input. Core code takes a `Clock`/`Timestamp`; only the live app
   edge reads the machine clock, and it records what it read. The M3 reorder
   stage and clock gate use only *receive* times, which are recorded.
2. The engine is single-threaded and a pure function of its ordered input.
3. No `-ffast-math`; `-ffp-contract=off` so FMA contraction cannot differ
   between builds (see `cmake/SiftedOptions.cmake`).
4. No iteration over address- or hash-ordered containers where order affects
   output; ties are broken by id.
5. Timestamps and ids are integers.
6. (M3) Every parameter that changes output is in `PipelineConfig` and in the
   event log header (ADR 0005). Impairments, faults and benchmark workloads
   are seeded (SplitMix64) and reproducible; the impairment model is
   bit-identical between C++ and Python (golden fixture).
7. (M3) Performance shortcuts must not change results: the gate screen is
   proven conservative and checked by tests and by the engine fuzzer against
   an engine without it.

Verified: clean, impaired (loss, bursts, jitter, duplicates, outage) and
faulted runs replay byte-identically (`EventLog.*`, `Reorder.ImpairedRecordedRunReplaysBitForBit`,
`Faults.FaultedRunsAreDeterministic`), and a live daemon run through the
impairment proxy with a sensor restart replayed identically (`cmp` of the
output streams, 2 056 lines, 117 frames lost to the outage, 89 to loss, 28
duplicates).

Known limit: results are guaranteed identical for the same binary and
architecture. Cross-compiler/cross-libm bit-equality of transcendental
functions (`sin`, `atan2`) is **not** promised; CI replays on the same build.

## 4. Modules

| Dir | Responsibility | Status |
|---|---|---|
| `core` | types, `Clock`, `Measurement` validation, `SensorTable`, multi-target `Engine` | done |
| `wire` | versioned, size-limited, strict frame codec | done (v1, fuzzed) |
| `estimation` | CV Kalman filter, range-bearing and bearing-only EKF models, geodetic projection, gate screen | done |
| `association` | gating, GNN assignment (and greedy, for comparison), lifecycle | done (ADR 0004) |
| `store` | track table, bounded decision trace, `explain()` | done |
| `replay` | event log, simulated-clock replayer | done (ADR 0005) |
| `net` | ingest pipeline (dedup, clock gate, reorder stage), UDP, rate limiting, queue, WebSocket, impairment model | done (ADRs 0006, 0008-0010) |
| `apps` | `sifted_fusiond` (live and replay) | done |
| `sim/` | simulators, scenarios, sender with fault injection, impairment proxy, OpenSky adapter | done |
| `benchmarks/` | engine harness, end-to-end latency, report | done (ADR 0011) |

## 5. Milestones

Scope is cut before polish. Each milestone ends with green CI and updated docs.

- **M0:** build, CI, sanitizers, fuzz harness, wire codec, docs.
- **M1:** radar simulator with ground truth; single-sensor tracker; sensor
  table (ADR 0003); first regression scenarios.
- **M2:** multi-sensor association + lifecycle; camera fusion; decision trace;
  UDP ingest; record and replay; WebSocket stream and Leaflet map; ADS-B.
- **M3:** degraded-network simulator and tests; fault injection; fuzzing of
  every parser and of the engine; benchmarks; design doc, threat model, demo
  runbook (the video itself is to be recorded from `docs/demo.md`).
- **Stretch:** CUDA association, only after M3.

## 6. Estimation and association

**Frame.** Local flat x/y plane in metres, +x east, +y north; angles are
radians counter-clockwise from +x. A sensor's pose (position, boresight yaw)
comes from the sensor table (ADR 0003); measured bearings are relative to the
boresight. Geodetic input is projected on a fixed per-run tangent plane
(ADR 0007).

**Filter.** `estimation::ConstantVelocityFilter`, state `[px, py, vx, vy]`,
continuous white-noise-acceleration process noise with PSD `q` per axis.
`predict(dt)` takes seconds explicitly. All updates go through one
Joseph-form `update_with_innovation(y, H, R)` with a Cholesky solve of `S`;
`P` is re-symmetrised after every step. Every operation validates its inputs
and either commits a finite result or leaves the state untouched.

**EKF models.** Range-bearing `h(x) = [r, wrap(atan2 - yaw)]`, bearing-only
`h(x) = wrap(atan2 - yaw)`, geodetic position linear in the tangent plane.
Bearing innovations are wrapped with `std::remainder(., 2*pi)`. Updates are
refused when the predicted range is under 1 m.

**Association and lifecycle (ADR 0004).** Per group (one sensor, one
timestamp): screen (validity, sensor, kind, duplicates, order), lifecycle at
the group time, chi-square gating in each sensor's space (p = 0.999), a gate
screen that skips provably-out-of-gate pairs (section 10), two-pass global
nearest neighbour (established tracks first), updates, then initiation from
position-observing leftovers under `max_tracks`. Tentative -> confirmed
(3 hits) -> coasting (2.5 s) -> dropped (10 s; tentative 3 s; or 1 km
position sigma). Manoeuvre handling by fading-memory NIS process-noise
switching.

## 7. Tuning and baseline accuracy

`EngineConfig::accel_psd` defaults to `q = 4 m^2/s^3` (rule of thumb
`q ~ (0.5..1) * a_max^2 * T` for 2-3 m/s^2 manoeuvres at 1 Hz):

| q [m^2/s^3] | straight_crossing RMSE / raw | turn RMSE / raw |
|---|---|---|
| 1 | 0.43 | 0.95 |
| 4 (default) | 0.50 | 0.72 |
| 16 | 0.59 | 0.73 |

Multi-target baselines (default configuration, direct engine feed; the
numbers each test prints; unchanged between the M2 test-only stand-ins and
the real Eigen/GoogleTest, and unchanged by every M3 change):

| scenario | per-target RMSE [m] | missed | ID sw. | false | fused / radar-only |
|---|---|---|---|---|---|
| crossing_pair | 8.39 8.63 | 0/172 | 0 | 0 | 8.51 / 11.18 |
| close_pass | 10.49 7.64 12.20 | 0/258 | 0 | 0 | 10.11 / 13.33 |
| enter_leave | 7.78 9.98 9.62 8.24 | 0/302 | 0 | 0 | 8.91 / 11.10 |
| formation_split (M3) | 11.54 11.58 10.42 | 0/258 | 0 | 0 | 11.18 / 27.78 |
| formation_split, greedy | max 35.49, mean 27.27 | 1/258 | 2 | 0 | |

formation_split separates GNN from greedy assignment; across other noise
seeds greedy broke identity in all and GNN in some (the geometry is
genuinely ambiguous), see `tests/multi_target_regression_test.cpp`.

### Filter consistency

Does the covariance mean what the map's 1-sigma circle claims?
`tests/consistency_test.cpp` measures NEES (`e^T P^-1 e` against truth;
4-dof full state and 2-dof position) and NIS (the engine's own gating `d^2`
of every associated update, read from the track trace). It uses the
RMSE's matched target-instants, past the same 10 s burn-in. The 95%
interval for an average of N samples is chi-square(N n) / N. Each scenario
is one run with strongly time-correlated samples, so on the fixtures that
interval is a guide, not an exact test (`tests/consistency.hpp`).

**Matched model, Monte Carlo** (200 independent runs; truth driven by the
filter's own `q = 4` white acceleration; one range-bearing radar at 5 km;
manoeuvre switching off). At t = 90 s: NEES4 posterior 4.02, prior 3.84,
NEES2 2.03, against the 95% interval [3.62, 4.40] for 4 dof and
[1.73, 2.29] for 2 dof. NIS is 2.001 over 16,184 updates, interval
[1.969, 2.031]. **The filter is consistent when its model is right.** These
values are asserted.

**Fixture scenarios** (default configuration; "upd." is an estimate at its own
update instant, "extr." is predicted forward or coasting; the interval is
for the updated NEES4):

| scenario | truth | NEES4 upd. (95%) | NEES4 extr. | NEES2 upd. | NIS range-bearing (exp. 2) | NIS bearing (exp. 1) |
|---|---|---|---|---|---|---|
| straight_crossing | straight | 1.65 [3.41, 4.64] | n/a | 1.24 | 1.52 | n/a |
| crossing_pair | straight | 2.29 [3.56, 4.47] | 1.97 | 1.71 | 1.91 | 0.85 |
| enter_leave | straight | 2.16 [3.65, 4.37] | 1.77 | 1.66 | 1.75 | 0.86 |
| turn | 3 deg/s turn | 3.79 [3.41, 4.64] | n/a | 2.30 | 2.29 | n/a |
| close_pass | 1 of 3 turns | 2.77 [3.63, 4.39] | 3.54 | 1.90 | 2.08 | 0.95 |
| formation_split | 2 of 3 turn | 2.86 [3.64, 4.38] | 2.64 | 1.88 | 1.80 | 0.84 |
| demo | 1 of 5 turns | 2.09 [3.77, 4.24] | 1.98 | 1.51 | 1.79 | 0.94 |

In straight flight the filter is **pessimistic**. NEES4 is about half its
expected value, while NEES2 and NIS are 10-25% low. The cause is model
mismatch, not a defect: the simulator flies exactly straight legs (zero
acceleration), while the filter budgets for white acceleration of PSD `q`. At
steady state about half of `P` is that budget, for any `q > 0`. A
sweep down to `q = 0.05` leaves the straight scenarios' NEES4 at 1.7-2.4,
and Monte Carlo with zero-acceleration truth gives 1.7-2.3 at every time.
Velocity is the most over-covered state. The 1-sigma position circle is only
3-27% too large in radius (`sqrt(2 / NEES2)`). In turns
the same `q` plus NIS switching makes the filter roughly consistent (turn:
3.79; close_pass's turning target: 4.48). Without switching, turn rises to
5.43, which is optimistic. Tests assert "not optimistic" for straight
scenarios, which are identified from the truth, and only loose sanity limits
for manoeuvring ones. No default is changed. An honest straight-flight
covariance needs a low-`q` model mixed with a manoeuvre model (IMM), not a
smaller `q`, which breaks turns (turn NEES4 9.0 at `q = 1`).

## 8. Parameters

Everything below is in `PipelineConfig` (and recorded in every event log
header) unless marked.

| Parameter | Default | Reasoning |
|---|---|---|
| `accel_psd` | 4 m²/s³ | section 7 |
| `initial_velocity_sigma` | 100 m/s | up to roughly airliner speed from one fix |
| `gate_probability` | 0.999 | a lost true association is a 1-in-1000 event; global assignment makes the wide gate cheap |
| `near_miss_factor` | 4 | trace only |
| `assignment` | gnn | greedy for comparisons only (M3) |
| manoeuvre `fading` / `detect_probability` / `accel_psd_factor` | 0.8 / 0.99 / 10 | ADR 0004, engine.hpp |
| `confirm_hits`, `tentative_timeout_s`, `coast_after_s`, `drop_after_s`, `max_position_sigma_m` | 3, 3 s, 2.5 s, 10 s, 1000 m | ~1 Hz sensors; lifecycle.hpp |
| `max_tracks` | 1024 | resource bound |
| trace sizes | 64 events/track, 64 dropped tracks, 1024 measurements, 32 sensors/track | bounded memory |
| `snapshot_interval_s` | 0.5 s (measurement time) | display rate |
| `dedup_window` | 256 per sensor | covers a scan and its retransmissions |
| `max_scan_size` | 256 | bounds one assignment problem |
| `reorder_window_s` | 0 (library), **0.5 s (daemon)** | ADR 0009, section 9 |
| `reorder_capacity` | 4096 frames | bounded |
| `max_time_ahead_s` | 0 (off) | ADR 0010; needs a common timebase; set ≤ the reorder window |
| `gate_screen` (not recorded) | on | never changes results (section 10) |
| rate limit (daemon only, not recorded) | 20 000/s, burst 4 096, 1 024 sources | ADR 0010; drops never reach the engine or the log |

## 9. Degraded-network behaviour

**Model (ADR 0008).** Per sensor link: scheduled outages, Gilbert-Elliott
burst loss, delay + uniform jitter (so reordering follows from jitter), and
duplication with an independent delay; seeded per (seed, sensor).

**Levels** (`tests/degraded_network_test.cpp`): mild = 1% loss, 20 + U(0, 50)
ms, 1% duplicates; moderate = 3% loss + bursts (0.01 / 0.25 / 0.6), 50 +
U(0, 400) ms, 2% duplicates; severe = 8% + bursts (0.03 / 0.2 / 0.8), 100 +
U(0, 800) ms, 5% duplicates.

**Scoring.** As in the M2 regression (CLEAR-MOT style; `tests/scenario_support.hpp`),
but frames go through `net::Pipeline` exactly as the daemon consumes them
(arrival order, receive time = arrival, a quiet tick after 100 ms), and each
truth instant is scored on what the operator would see at that moment. Even a
clean network therefore scores slightly worse than the direct engine feed:
the radar scan stamped T is still being assembled at T.

**Out-of-order policy (ADR 0009).** Sweep over crossing_pair, close_pass and
enter_leave, three impairment seeds each (9 runs per row; ID switches of
formation_split, which is excluded from the bounded set, in parentheses):

| level | window s | out-of-order % | missed % | mean RMSE m | worst RMSE m | ID switches | false tracks |
|---|---:|---:|---:|---:|---:|---:|---:|
| clean | 0.00 | 0.0 | 0.00 | 11.10 | 18.69 | 0 (0) | 0 |
| clean | 0.25 | 0.0 | 0.00 | 11.45 | 19.18 | 0 (0) | 0 |
| clean | 0.50 | 0.0 | 0.00 | 11.45 | 19.18 | 0 (0) | 0 |
| clean | 1.00 | 0.0 | 0.23 | 13.99 | 27.73 | 0 (0) | 0 |
| clean | 2.00 | 0.0 | 0.57 | 17.11 | 37.32 | 0 (0) | 0 |
| mild | 0.00 | 0.0 | 0.00 | 11.16 | 18.87 | 0 (0) | 0 |
| mild | 0.50 | 0.0 | 0.00 | 11.52 | 19.36 | 0 (2) | 0 |
| mild | 1.00 | 0.0 | 0.23 | 14.06 | 27.97 | 0 (0) | 0 |
| moderate | 0.00 | 10.1 | 0.43 | 12.64 | 21.64 | 1 (4) | 0 |
| moderate | 0.25 | 0.0 | 0.32 | 12.02 | 20.67 | 1 (2) | 0 |
| moderate | 0.50 | 0.0 | 0.32 | 12.12 | 20.81 | 1 (4) | 0 |
| moderate | 1.00 | 0.0 | 0.59 | 14.73 | 29.92 | 1 (2) | 0 |
| moderate | 2.00 | 0.0 | 1.13 | 17.84 | 39.93 | 1 (2) | 0 |
| severe | 0.00 | 29.0 | 6.52 | 15.90 | 29.85 | 6 (9) | 0 |
| severe | 0.25 | 6.5 | 0.68 | 13.88 | 27.14 | 1 (9) | 0 |
| severe | 0.50 | 0.1 | 0.52 | 13.87 | 29.37 | 1 (0) | 0 |
| severe | 1.00 | 0.0 | 0.96 | 15.54 | 32.06 | 1 (2) | 0 |
| severe | 2.00 | 0.0 | 1.80 | 18.62 | 40.15 | 1 (2) | 0 |

Decision: a bounded reorder stage, 0.5 s in the daemon. It recovers
essentially all the data the M2 drop policy loses (29% -> 0.1% at severe)
for +0.35 m mean RMSE and +0.4 s of end-to-end latency on a clean network
(measured: 101 ms -> 502 ms at 100 targets, docs/benchmarks.md).
The engine still drops whatever arrives later than the window (counted).

**Recovery after an outage.** Definition (`recovery_after`): the targets
covered and matched just before an outage [start, end) of every link and
still covered after it; a target has recovered at the first truth instant
≥ end at which it is matched to a track *updated after the outage ended*
(extrapolation through the gap does not count); recovery time is from the
outage end until all have recovered; a target "survived" if that track is the
one it had before. Measured (mild level, outage at t = 30 s, both windows):

| outage | crossing_pair | close_pass | enter_leave |
|---|---|---|---|
| 5 s | 1 s, 2/2 survived | 3 s, 2/3 (the turning target re-initiated) | 1 s, 3/3 |
| 8 s | 1 s, 2/2 | 3 s, 2/3 | 3 s, 2/3 |
| 15 s | 3 s, 0/2 (dropped as Stale, re-initiated) | 4 s, 0/3 | 4 s, 0/3 |

Rule the test asserts generally: if the gap from a track's last update to
the outage end plus two revisits is within `drop_after_s` and the target
flies straight, the track survives; if the gap exceeds `drop_after_s` it is
re-initiated; recovery within 5 s (+ window) either way. The demo scenario
(8 s outage at the moderate level) recovers in 4 s with all five targets
tracked (`DegradedNetwork.DemoScenarioAsInTheRunbook`).

## 10. Performance

Full report: [`benchmarks.md`](benchmarks.md). Summary (sandbox VM, 2 vCPU
Xeon, GCC 13 `-O3`, 5 runs per configuration):

- per-scan engine time at 1 000 targets + 10% clutter: median 10.0 ms, p99
  10.5 ms (before the gate screen: median 145 ms);
- throughput, one engine thread, 300 targets + 30 clutter/scan:
  ~309 000 measurements/s;
- **tracks per core: 2 976 targets (10% clutter, ~4 150 live tracks
  including tentative ones) with p99 scan time ≤ 100 ms at 1 Hz;**
- end to end through the real daemon (UDP send to WebSocket snapshot,
  loopback): median 101 ms at 100 targets and 115 ms at 1 000 with the
  reorder window off -- the 100 ms is the quiet tick that ends a scan -- and
  502 / 519 ms with the default 0.5 s window.

Profile (callgrind): before M3, > 70% of engine time at 1 000 targets was the
all-pairs gating computation. The **gate screen** (`estimation::may_gate`)
bounds `d²` from below with quantities that cost a subtraction, a norm and a
few multiplications per pair: for SPD `S`, `d² ≥ y_i²/S_ii`; for
range-bearing `S_rr ≤ λ + σ_r²`, `S_bb ≤ λ/r² + σ_b²` (λ: largest eigenvalue
of the predicted position covariance) and the plane distance satisfies
`D² ≤ y_r² + r_m r y_b²`; so `d² ≤ G` implies
`D² ≤ G (λ (1 + r_m/r) + σ_r² + r_m r σ_b²)`. Pairs beyond that (widened by
1e-6 relative) are skipped. It is exact in effect: every regression,
degraded, fault and replay test is unchanged, `GateScreen.*` compares full
traces with the screen on and off, and `fuzz_engine` does the same on
arbitrary input. The remaining cost is the all-pairs screen itself
(O(measurements x tracks)); a spatial index would remove it and is deferred.

## 11. Open questions (final status)

- Local tangent-plane origin for ADS-B: **resolved**, fixed per run and
  recorded (ADR 0007).
- Out-of-order measurements: **resolved**, bounded reorder stage before the
  engine, drop beyond it (ADR 0009). Retrodiction deferred (not needed at the
  measured jitter levels).
- WebSocket library vs. minimal server: **resolved**, minimal read-only
  server, fuzzed (ADR 0006).
- Clock faults: **resolved** with the opt-in clock gate (ADR 0010).
- Sensor bias estimation / registration, frozen-sensor detection, spatial
  indexing for >3 000 tracks per thread, CUDA association: **deferred**
  (documented limitations).
