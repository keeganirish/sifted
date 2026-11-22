# Sifted design

> Status: complete through M3. Decisions are recorded in [`adr/`](adr/)
> (0001-0013). Measured numbers come from the tests and benchmarks named next
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
             decode -> timeline guard -> dedup (seq, time) -> clock gate -> reorder stage -> scan assembly
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
| recording sync (`DurableFile`, only with `--record`) | nothing | calls `fdatasync` on the log's descriptor about once a second, so a slow disk never stalls the engine thread |

The engine thread is the only one that touches tracker state. The receive
thread reads the wall clock (receive stamps) and the engine thread reads it
only to stamp ticks; both values go into the event log, so replay never reads a
clock. The engine thread also repeats the latest snapshot once a second while
idle. That heartbeat is not recorded and is not part of the output: it changes
only what a connected client knows about the daemon being alive.

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
7. Idle time reaches the pipeline only as recorded ticks. Ticks can change what
   is displayed, never what is tracked: predicted snapshots use a read-only view
   of the lifecycle (ADR 0012, `Silence.IdleTicksNeverChangeWhatIsTracked`).
8. (M3) Performance shortcuts must not change results: the gate screen is
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

**Motion model (ADR 0014).** New tracks get, by default, an Interacting
Multiple Model estimator (`estimation::ImmFilter`). It runs three CV modes in
the same state, with q = 0.5 / 25 / 1200 m²/s³ (quiet, manoeuvre, hard), and
switches between them by a continuous-time Markov chain with mean sojourns of
60 / 30 / 15 s, `Π(dt) = expm(G dt)`. Each cycle mixes the modes (with the
spread-of-means term), predicts each mode with its own q, applies a
Joseph-form update to each mode linearised at its own state, and weights the
modes by their Gaussian likelihoods in log space, floored at 1e-4. The output
is the moment-matched combination. `dt = 0` is an exact no-op, and an update
is all or nothing across modes. `state()`/`covariance()` of a track, and so
`TrackEstimate`, the snapshot and the scorer, are the combined 4-state
estimate. The legacy single CV filter with NIS switching (`motion_model =
cv`) remains selectable and is what every recording made before ADR 0014
replays with. `estimation::TrackFilter` gives the engine one interface to
both.

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
position sigma). For the IMM the gating statistic and association cost is
the mixture statistic `-2 ln Σ w_j exp(-d_j²/2)`. That is the IMM's
predicted measurement density, relative to its peak scale, over the modes'
own `d_j²`. For one component it is exactly the CV filter's `d²` (ADR 0014
compares it with the alternatives). Manoeuvre indication: the legacy filter
switches process noise on a fading-memory NIS sum; the IMM flags
`manoeuvring` when `1 - μ_quiet` exceeds 0.8, until it falls below 0.4.

## 7. Tuning and baseline accuracy

`EngineConfig::accel_psd` defaults to `q = 4 m^2/s^3` (rule of thumb
`q ~ (0.5..1) * a_max^2 * T` for 2-3 m/s^2 manoeuvres at 1 Hz):

| q [m^2/s^3] | straight_crossing RMSE / raw | turn RMSE / raw |
|---|---|---|
| 1 | 0.43 | 0.95 |
| 4 (default) | 0.50 | 0.72 |
| 16 | 0.59 | 0.73 |

Multi-target baselines (default configuration, direct engine feed: per-target
RMSE, missed, ID switches, false tracks, fused against radar-only RMSE) are
generated, not copied: see the "Direct engine feed" table of
[evaluation.md](evaluation.md), written by `sifted_eval`. They are unchanged
since M2, and `Eval.ScorecardMatchesRegressionTests` checks that the tool
reproduces the baselines recorded in `tests/multi_target_regression_test.cpp`.

formation_split (M3) separates GNN from greedy assignment: greedy scores max
35.49 m / mean 27.27 m RMSE, 1/258 missed, 2 ID switches on it (the tool runs
the default GNN only). Across other noise seeds greedy broke identity in all
and GNN in some (the geometry is genuinely ambiguous), see
`tests/multi_target_regression_test.cpp`.

These fixture numbers are one noise realisation each. The statistical picture
is [evaluation-monte-carlo.md](evaluation-monte-carlo.md) (ADR 0015): 100
independent realisations per scenario and a clutter × Pd sweep, with bootstrap
intervals, the fraction of runs with any identity break, and GOSPA (alpha = 2,
c = 100 m, p = 2; `eval/sifted/eval/gospa.hpp`) beside the CLEAR-MOT metrics.
[evaluation-baseline.md](evaluation-baseline.md) compares Sifted with a
Stone Soup tracker on the same runs and scorer. The scorer accepts any
tracker's output as a track report (`eval/sifted/eval/track_report.hpp`); NIS
and the engine counters are then marked unavailable.

### Filter consistency

Does the covariance mean what the map's 1-sigma circle claims?
`tests/consistency_test.cpp` measures NEES (`e^T P^-1 e` against truth;
4-dof full state and 2-dof position) and NIS (the engine's own gating `d^2`
of every associated update, read from the track trace). It uses the
RMSE's matched target-instants, past the same 10 s burn-in. The 95%
interval for an average of N samples is chi-square(N n) / N. Each scenario
is one run with strongly time-correlated samples, so on the fixtures that
interval is a guide, not an exact test (`eval/sifted/eval/consistency.hpp`).

**Matched model, Monte Carlo** (200 independent runs; truth driven by the
filter's own `q = 4` white acceleration; one range-bearing radar at 5 km;
manoeuvre switching off). At t = 90 s: NEES4 posterior 4.02, prior 3.84,
NEES2 2.03, against the 95% interval [3.62, 4.40] for 4 dof and
[1.73, 2.29] for 2 dof. NIS is 2.001 over 16,184 updates, interval
[1.969, 2.031]. **The filter is consistent when its model is right.** These
values are asserted.

**Fixture scenarios** (default configuration, direct engine feed): the NEES
and NIS of every fixture are in the "Filter consistency" table of
[evaluation.md](evaluation.md), generated by `sifted_eval`. Truth is straight
flight in straight_crossing, crossing_pair and enter_leave; turn flies a
3 deg/s turn; one of close_pass's three targets turns, two of
formation_split's, one of demo's five.

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
for manoeuvring ones. An honest straight-flight covariance is not available to
any filter with a non-zero steady-state gain against zero-acceleration truth,
and a smaller `q` breaks turns (turn NEES4 9.0 at `q = 1`). The paragraphs
above describe the legacy CV filter.

**IMM (ADR 0014, now the default).** Measured by `sifted_motion_eval`
([motion-model-evaluation.md](motion-model-evaluation.md)), legacy against
IMM, NEES4 by flight phase over the 110-run manoeuvre set:

| phase | legacy | IMM |
|---|---:|---:|
| straight | 2.57 | 2.10 |
| turn onset (first 10 s) | 86.4 | 16.8 |
| turn | 32.4 | 2.73 |
| turn exit (first 10 s) | 4.83 | 2.11 |

The IMM fixes the manoeuvre inconsistency (in turns it is now slightly
pessimistic instead of 8x optimistic). It does **not** make straight flight
honest. The cause is the one above: the simulator's straight legs have
exactly zero acceleration, and the manoeuvre modes the IMM mixes in add
process noise that is not there. Onset stays optimistic for the few scans the
mode probabilities need to move.

On its own matched model (`ConsistencyMonteCarlo.ImmMatchedModelIsNearlyConsistent`:
truth switching modes with the IMM's own `Π(1 s)` and driven by each mode's
white acceleration; 1 000 runs, estimator fed every measurement), the IMM's
pooled NEES4 is 4.22 prior / 4.15 posterior, and NIS 2.002 per 2 dof. That
is 4-5% optimistic, as expected of moment matching, and asserted within 15%.
Through the engine the same truth loses its track (re-initiation) in 54 of
200 runs. The mixture gate discounts detections from a mode the track thinks
improbable, which is the price of not chasing neighbours in a formation
(ADR 0014).

## 8. Parameters

Everything below is in `PipelineConfig` (and recorded in every event log
header) unless marked.

| Parameter | Default | Reasoning |
|---|---|---|
| `motion_model` | `imm` (`cv` when a log header lacks it) | ADR 0014 |
| `imm_accel_psd_quiet` / `_manoeuvre` / `_hard` | 0.5 / 25 / 1200 m²/s³ | `q ≈ (0.5..1) a² T` for ~1 m/s², 5 m/s², 5 g; ADR 0014 |
| `imm_sojourn_quiet_s` / `_manoeuvre_s` / `_hard_s` | 60 / 30 / 15 s | straight legs of minutes, turns of tens of seconds; ADR 0014 |
| `imm_probability_floor` | 1e-4 | recovery needs one update with d² ≈ 18; ADR 0014 |
| `imm_manoeuvre_on` / `_off` | 0.8 / 0.4 | indication hysteresis on `1 - μ_quiet`; display and trace only |
| `accel_psd` | 4 m²/s³ | legacy `cv` model only; section 7 |
| `initial_velocity_sigma` | 100 m/s | up to roughly airliner speed from one fix |
| `gate_probability` | 0.999 | a lost true association is a 1-in-1000 event; global assignment makes the wide gate cheap |
| `near_miss_factor` | 4 | trace only |
| `assignment` | gnn | greedy for comparisons only (M3) |
| manoeuvre `fading` / `detect_probability` / `accel_psd_factor` | 0.8 / 0.99 / 10 | ADR 0004, engine.hpp |
| `confirm_hits`, `tentative_timeout_s`, `coast_after_s`, `drop_after_s`, `max_position_sigma_m` | 3, 3 s, 2.5 s, 10 s, 1000 m | ~1 Hz sensors; lifecycle.hpp |
| `max_tracks` | 1024 | resource bound |
| trace sizes | 64 events/track, 64 dropped tracks, 1024 measurements, 32 sensors/track | bounded memory |
| `snapshot_interval_s` | 0.5 s (measurement time) | display rate |
| `snapshot_trace_events` | 4 per track (at most 32) | display only; see below |
| `dedup_window` | 256 per sensor | covers a scan and its retransmissions |
| `max_scan_size` | 256 | bounds one assignment problem |
| `reorder_window_s` | 0 (library), **0.5 s (daemon)** | ADR 0009, section 9 |
| `reorder_capacity` | 4096 frames | bounded |
| `max_time_ahead_s` | 0 (off) | ADR 0010; needs a common timebase; set ≤ the reorder window |
| `max_time_jump_s` | 0 (library), **`coast_after_s` (daemon)** | ADR 0012: a jump that small cannot even make a track coast |
| `timeline_confirm_frames` | 3 | ADR 0012: distinct frames needed to adopt a new timeline |
| `timeline_confirm_s` | 2.5 s (library), **`coast_after_s` (daemon)** | ADR 0012: silence on the old timeline before it can be replaced |
| `feed_stale_factor` | 0 (library), **3 (daemon)** | ADR 0012: tolerates two missed scans of a sensor's own median cadence |
| `feed_stale_min_s` | 2.5 s (library), **`coast_after_s` (daemon)** | ADR 0012: never call a sensor stale before its tracks would coast |
| idle tick, recording flush and sync (daemon only, not recorded) | 1 s; 100 ms / 64 records; 1 s | ADR 0012; ticks are themselves recorded, and the rest cannot change output |
| `gate_screen` (not recorded) | on | never changes results (section 10) |
| rate limit (daemon only, not recorded) | 20 000/s, burst 4 096, 1 024 sources | ADR 0010; drops never reach the engine or the log |

**Decision trace in snapshots.** The trace records why the system believes a
track: its birth, each association with d² and the gate, contested
measurements, near misses, rejected updates, and lifecycle and manoeuvre
changes. It is bounded so memory and message size stay fixed however long a
track lives. Snapshot v2 (`net/snapshot.hpp`) carries each track's newest
`snapshot_trace_events` decisions and `trace_total`, the number of decisions
ever made, so the operator view can join successive excerpts without
duplicates and keep up to 64 per track, discarding routine associations first.
The stream stays one-way (ADR 0006) and a replay publishes the same excerpts,
so the page's history starts when it connects. Dropped tracks leave the
snapshot, so a `dropped` event is encoded by the format but is not normally
published.

Four events is the shortest excerpt that covers nearly every gap between two
snapshots: in six recorded demo runs and two synthetic runs (100 and 1 000
targets), 99.97% of 0.5 s intervals added at most four decisions to a track.
Sending only new events was rejected because the publisher deliberately lets
slow clients skip snapshots, so a skipped delta would be lost. Cost, measured
with `benchmarks/e2e_latency.py` workloads against the build without the trace:

| targets | snapshot before | snapshot with 4 events | median e2e latency before | with 4 events |
|---:|---:|---:|---:|---:|
| 100 | 35 KB | 100 KB | 101.6 ms | 101.8 ms |
| 1 000 | 352 KB | 1.00 MB | 113.1 ms | 118.1 ms |

## 9. Degraded-network behaviour

**Model (ADR 0008).** Per sensor link: scheduled outages, Gilbert-Elliott
burst loss, delay + uniform jitter (so reordering follows from jitter), and
duplication with an independent delay; seeded per (seed, sensor).

**Levels** (`eval/sifted/eval/conditions.cpp`, shared by the tests and
`sifted_eval`): mild = 1% loss, 20 + U(0, 50) ms, 1% duplicates; moderate =
3% loss + bursts (0.01 / 0.25 / 0.6), 50 + U(0, 400) ms, 2% duplicates;
severe = 8% + bursts (0.03 / 0.2 / 0.8), 100 + U(0, 800) ms, 5% duplicates.

**Current results** for every scenario, level, outage and fault case are in
[evaluation.md](evaluation.md), generated by `sifted_eval`; this section
keeps only the measurements that drove a decision.

**Scoring.** As in the M2 regression (CLEAR-MOT style; `eval/sifted/eval/scenario.hpp`),
but frames go through `net::Pipeline` exactly as the daemon consumes them
(arrival order, receive time = arrival, a quiet tick after 100 ms), and each
truth instant is scored on the tracks the pipeline holds at that moment,
extrapolated to it. That is the tracker's estimate, not the last message an
operator received: snapshots are published at most every 0.5 s and, in a silent
stretch, only once the feed is declared stale (section 9, "Silence"). Even a
clean network scores slightly worse than the direct engine feed: the radar scan
stamped T is still being assembled at T.

**Out-of-order policy (ADR 0009).** Sweep over crossing_pair, close_pass and
enter_leave, three impairment seeds each (9 runs per row; ID switches of
formation_split, which is excluded from the bounded set, in parentheses).
Recorded when the decision was made; `sifted_eval` regenerates it (the
"Summary" rows; formation_split's ID switches by adding its directory):

```bash
F=tests/fixtures/scenarios
build/release/eval/sifted_eval --scenarios $F/crossing_pair --scenarios $F/close_pass \
    --scenarios $F/enter_leave --window 0,0.25,0.5,1,2
```

It reproduces every RMSE, ID-switch and out-of-order figure below; missed %
comes out within 0.02 of it, because this table averaged the 3-decimal
ratios the tests print.

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

The table was measured with the legacy constant-velocity filter. The same
command under the IMM (ADR 0014) gives, at the severe level, 29.0% → 0.1% out
of order, missed 6.79% → 0.53% and 7 → 1 ID switches without and with the
0.5 s window, at +0.27 m mean RMSE on a clean network. The decision stands.

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
one it had before. Measured recovery times and survivors per scenario
(mild level, every link down from t = 30 s for 5, 8 and 15 s) are the
"outage" rows of [evaluation.md](evaluation.md). In short: a 5 s outage is
recovered in 1-3 s with straight-flying targets on their old tracks (a
target turning through the gap is re-initiated); after 15 s every track has
been dropped as Stale and re-initiated, within 3-4 s.
`DegradedNetwork.RecoveryAfterOutage` uses seed 11 and both windows; add
`--seeds 11 --levels mild --window 0,0.5` to reproduce its numbers.

Rule the test asserts generally: if the gap from a track's last update to
the outage end plus two revisits is within `drop_after_s` and the target
flies straight, the track survives; if the gap exceeds `drop_after_s` it is
re-initiated; recovery within 5 s (+ window) either way. The demo scenario
(8 s outage at the moderate level) recovers in 4 s with all five targets
tracked (`DegradedNetwork.DemoScenarioAsInTheRunbook`).

**Timeline integrity (ADR 0012).** The daemon quarantines frames that are more
than `max_time_jump_s` off the current timeline, beyond the receive time elapsed.
One forged frame is rejected and counted. A real change of timeline (a sensor
restarting on a new epoch, every sensor moving forward) is adopted after 3 distinct
frames and `timeline_confirm_s` of silence on the old timeline, without losing the
held frames. Gaps covered by elapsed receive time pass unquestioned: 15 s outages at
real time and ADS-B polls 22-216 s apart. A sender running faster than real time
across a gap (`--speed 0` over a 15 s hole) pauses for the confirmation time.

**Silence (ADR 0012).** A sensor is stale after `max(feed_stale_min_s,
feed_stale_factor x its median inter-scan gap)`. Every snapshot says which sensors
are live: `live`, `degraded` (some silent), `stale` (all silent). During total
silence, recorded idle ticks publish predicted snapshots at most once a second, in
which confirmed tracks are shown coasting after `coast_after_s` and disappear after
`drop_after_s`. The tracker itself is not touched: when data returns, the engine's
lifecycle decides exactly as it would have, so the outage results in
[evaluation.md](evaluation.md) are unchanged. Ticks stop once the picture is empty.
The operator view adds its own watchdog for a hung service (5 s without any message,
when the daemon's heartbeat is one per second).

**Crash durability (ADR 0012).** Process death loses at most the newest 64 records
or about 200 ms of input; power loss about 1.2 s. A torn final record is recoverable
with `--recover-truncated`; other damage remains a hard error. A crashed run cannot
be verified in full, so Verify compares what the run and its recording both saved
and says "partial".

## 10. Performance

Full report: [`benchmarks.md`](benchmarks.md). Summary (sandbox VM, 2 vCPU
Xeon, GCC 13 `-O3`, 5 runs per configuration, the legacy constant-velocity
filter; the IMM's relative cost follows the list):

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

**IMM (ADR 0014), measured against the legacy filter on one machine** (a
laptop, not the sandbox; benchmarks.md, "Motion models"): 2 898 against
2 976 tracks per core (97%). Per-scan time is +5% to +27% at 1 000-3 000
targets, where the all-pairs screen dominates. Small pictures and the
300-target throughput run, where estimation dominates, cost 1.6-2.7x
(213 000 against 327 000 measurements/s). The IMM's screen is one envelope
test per pair covering every mode (`estimation::may_gate_envelope`), and
each track is predicted once per group. Before both changes the IMM reached
1 532 tracks per core.

Profile (callgrind): before M3, > 70% of engine time at 1 000 targets was the
all-pairs gating computation. The **gate screen** (`estimation::may_gate`)
bounds `d²` from below with quantities that cost a subtraction, a norm and a
few multiplications per pair: for SPD `S`, `d² ≥ y_i²/S_ii`; for
range-bearing `S_rr ≤ λ + σ_r²`, `S_bb ≤ λ/r² + σ_b²` (λ: largest eigenvalue
of the predicted position covariance) and the plane distance satisfies
`D² ≤ y_r² + r_m r y_b²`; so `d² ≤ G` implies
`D² ≤ G (λ (1 + r_m/r) + σ_r² + r_m r σ_b²)`. Pairs beyond that (widened by
1e-6 relative) are skipped. With the IMM a track has three Gaussian
components (the mode-matched predictions), and its statistic is
`d²_mix = -2 ln Σ w_j exp(-d_j²/2)` with `w_j ≥ 0`, `Σ w_j = 1`, so
`d²_mix ≥ min_j d_j²`. This is clamped in the implementation, so rounding
cannot break it. Hence `d²_mix ≤ G` implies `d_j² ≤ G` for some mode, which
by the bound above implies the screen passes for that mode's position and λ.
A pair may therefore be skipped only when the screen fails for every mode.
The engine tests all modes at once with an envelope (`may_gate_envelope`):
centre `c` (the combined position), spread `ρ` (largest distance of a mode
from it) and the largest mode variance. Every mode's distance is at least
`D_c - ρ`, its range lies in `[r_c - ρ, r_c + ρ]`, and the bound grows with λ
and is convex in `r`, so it is largest at an end of that interval. The pair is
skipped if `D_c² > 2B + 2ρ²`, which implies `D_c - ρ > √B` and so that every
mode fails. It is exact in effect: every regression,
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
- Clock faults: **resolved** with the opt-in clock gate (ADR 0010) and the default
  timeline guard (ADR 0012).
- Manoeuvring targets and an honest covariance: **resolved for manoeuvres**
  with the IMM (ADR 0014): ID switches 0.0-0.9 per run through 5 g, against
  2-7 before. **Open:** straight-flight NEES stays about half its expected
  value against the simulator's zero-acceleration legs. Dense formations
  (45 m spacing) lose identity more often with the IMM than with the legacy
  filter. A turn mode with estimated rate, or formation-aware association,
  would be the next steps.
- Sensor bias estimation / registration, frozen-sensor detection, spatial
  indexing for >3 000 tracks per thread, CUDA association: **deferred**
  (documented limitations).
