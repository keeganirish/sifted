# ADR 0014: Interacting Multiple Model estimator -- three CV modes, mixture gating, default for new runs

Status: accepted

## Context
Each track used one constant-velocity (CV) Kalman filter with one process
noise (`accel_psd` = 4 m²/s³) and, when a fading-memory NIS sum crossed a
chi-square threshold, ten times that noise (ADR 0004, design.md section 6).
The single filter is a compromise. It is pessimistic in straight flight
(NEES4 about 2 of 4), and it loses targets that turn harder than it budgets
for. A stress test of the existing simulator (one radar, 1 Hz, Pd 0.9, 0.5
clutter per scan, 60 km coverage; 30 s straight, a turn of
`min(180°/rate, 40 s)`, 30 s straight; 10 seeds) gave 1.4-6.9 ID switches per
run and 2-14% missed target-instants from 1 g up. The committed fixtures turn
at 0.3 g at most and never showed it.

## Decision

### Modes: three nearly-constant-velocity models in one state
`estimation::ImmFilter` (imm_filter.hpp) runs three CV models in the same
state `[px, py, vx, vy]`. They differ only in white-noise acceleration PSD.
Each value is chosen from the usual rule `q ≈ (0.5..1) a² T` at T = 1 s,
before any evaluation run:

| mode | q [m²/s³] | reasoning |
|---|---:|---|
| quiet | 0.5 | ~1 m/s² of gusts and speed/height-keeping corrections in straight flight |
| manoeuvre | 25 | a = 5 m/s², factor 1: ordinary turns (standard rate at 250 m/s is ~4 m/s²) |
| hard | 1200 | a = 5 g, factor 0.5: high-g turns of agile aircraft |

Alternatives rejected:
- **CV + coordinated turn with estimated turn rate (5 states).** The usual
  choice in the literature, and the most accurate in a *sustained* turn. At
  1 Hz with 15-90 m cross-range noise the turn rate takes several scans to
  converge, and it is unobservable from bearing-only (camera) updates, which
  carry no range. The rate state would also need augmentation and
  de-augmentation at every mixing step (5-D vs 4-D modes), plus a small-rate
  Jacobian. Most of all, a turn at an onset that the turn-rate estimate has
  not yet caught still needs a high-noise mode. That makes three modes anyway.
- **A bank of fixed turn rates.** Covering ±28°/s (5 g at 100 m/s) in both
  directions at useful resolution needs 8+ modes, so 8x the work per track.
- **Two modes (CV + high noise).** With one high-noise mode sized for 5 g,
  gentle turns run on a very noisy model and the committed turn fixture's
  RMSE regresses. A mode sized for gentle turns loses 5 g targets: the steady
  lag of a CV filter with q = 200 in a 49 m/s² turn is ~3.5σ of its innovation.

Same-dimension modes keep mixing exact (moment matching, no truncation or
augmentation). `Q(dt)` stays closed form for any asynchronous interval. Every
mode uses the shared range-bearing, bearing-only and geodetic models
unchanged, each linearised at its own state (bearing wrap included).

### Transition model: a continuous-time Markov chain
Mode i is left at rate 1/τ_i, to either other mode with equal probability. The
generator is `G_ii = -1/τ_i`, `G_ij = 1/(2 τ_i)`, and
`Π(dt) = expm(G dt)` (degree-12 Taylor series with exact power-of-two scaling
and squaring, plain IEEE arithmetic, so deterministic). `Π` depends on
elapsed time, is a semigroup, and `Π(0) = I`. `predict(0)` is short-circuited
to an exact no-op, because several sensor groups can share a timestamp. The
stationary distribution is ∝ τ. Sojourn times (quiet / manoeuvre / hard):
60 / 30 / 15 s. Straight legs last minutes, and a high-g turn of 180° lasts
10-20 s. Considered and rejected: `π_ii = exp(-dt/τ_i)` with fixed
off-diagonals. It is simpler but not a semigroup, so a track updated at 2 Hz
and one updated at 1 Hz would follow different priors.

### The cycle, robustness and edge cases
- Mixing `μ_i|j = Π_ij μ_i / c_j`; mixed `x0_j`, `P0_j` with spread-of-means;
  mode-matched prediction; Joseph-form mode-matched updates with
  symmetrisation; output `x = Σ μ_j x_j`, `P = Σ μ_j (P_j + d_j d_jᵀ)`. Every
  result is computed into temporaries and committed only if finite.
- Mode likelihoods are full Gaussian densities `N(y_j; 0, S_j)` in the
  measurement's own space: a 1-D bearing and a 2-D position alike. Only ratios
  between modes matter, so the dimension constant cancels. They are combined
  in log space, shifted by the maximum. If the normalisation is still not
  finite and positive, the predicted probabilities are kept (deterministic).
- Probability floor `1e-4` after every update, then renormalisation, so no
  mode becomes unreachable. Same-time updates give the chain no transition
  mass. Recovering from the floor needs a likelihood ratio of 10⁴ (~9 nats,
  d² ≈ 18): one update well outside the gate.
- **All or nothing.** If any mode cannot be linearised (a mode predicted
  within 1 m of the sensor) or its update is not finite, the whole update is
  rejected (`FilterRejected`, traced). Updating only some modes would make
  the mixture mean silently depend on which modes happened to fail.
- **Coasting.** With no measurement, the probabilities follow the chain
  (`c = Πᵀ μ`) and relax to the stationary distribution over minutes. The
  combined covariance grows accordingly.
- **Start.** Every mode gets the single-fix state and covariance (velocity
  prior 100 m/s), with the stationary probabilities: one fix says nothing about
  how the target moves. Starting in the quiet mode instead was tried and made
  no material difference (formation selection set 0.7 vs 0.8 ID switches per run).

### Gating statistic: the mixture density
`TrackFilter::gate` returns
`d²_mix = -2 ln Σ_j w_j exp(-d_j²/2)`, `w_j = c_j |S_j|^-½ / Σ_k c_k |S_k|^-½`:
-2 ln of the IMM's predicted measurement density, relative to its value if
every mode predicted the measurement exactly. Each `d_j²` and `S_j` comes from
the mode's own linearisation and covariance. For one component it is exactly
the legacy `d²`. The association cost is the same statistic, because the
solver only takes a pair cheaper than the gate. The gate screen tests every
mode and stays provably conservative: `d²_mix ≥ min_j d_j²`, clamped against
rounding (proof in measurement_model.hpp and design.md section 10). The
`Associated` trace still records, as `nis`, the d² under the combined
estimate, so consistency is reported on the IMM's output, not on its gate.

Alternatives, measured on two selection sets that are not the reported ones:
ten re-seeded copies of every multi-target fixture, and the manoeuvre tuning
set (left turns and S-turns at 1.5 and 4 g, 150 and 200 m/s, seeds 901-905).

| statistic | formation_split re-seeded, ID sw/run | tuning set, ID sw/run | why not |
|---|---:|---:|---|
| combined moment-matched estimate | 0.9 | 0.6-2.2 by condition | at a hard onset the quiet mode dominates the combined covariance: the first manoeuvring detections fall outside it, the modes never see the evidence, and the track is re-initiated |
| min over modes (union of validation regions) | 2.0 | 0.00 | after a gap the hard mode's prediction covers a 45 m formation, so the cost cannot tell a track's own detection from its neighbour's |
| min over modes of d² + 2 ln(c_max/c_j) | 1.2 | 0.20 | ignores the hard mode's spread |
| **mixture density (chosen)** | **0.8** | **0.48** | the IMM's own predictive density; best in dense traffic |
| legacy CV, for reference | 0.3 | 4.6-10.2 by condition | |

Not adopted either: excluding bearing-only updates from the mode
probabilities (selection set: 1.5 ID switches per run, worse). The bearing
likelihoods help separate the modes.

### Manoeuvre indication
`manoeuvring` = `1 - μ_quiet` above 0.8 (4:1 for a manoeuvre), cleared below
0.4 (2:3). The hysteresis band keeps evidence near one threshold from making
the flag flicker. It is evaluated after each applied update of a non-tentative
track; before confirmation the velocity, and so the motion, is not yet
observable. Changes are traced as the new kinds `manoeuvre likely` /
`manoeuvre unlikely`, with the probability and the threshold crossed, so the
trace stays bounded (one event per change) and deterministic. The legacy
`manoeuvre start/end` events are unchanged.

**Snapshot.** No schema change: the new kinds are additive text in `trace`,
and the version stays 2, so legacy runs keep their bytes. Mode probabilities
are not published: the flag and its trace events carry what an operator acts
on, and three numbers per track per snapshot would add bytes without a
decision behind them. They are available in `TrackEstimate::mode_probabilities`.

### Configuration, recording and the default
- `EngineConfig::motion_model` (`cv` | `imm`) and `EngineConfig::imm` (PSDs,
  sojourns, floor, indication thresholds) are validated (`is_valid`) and
  recorded in every event-log header (`motion_model`, `imm_*`). Daemon:
  `--motion-model`, `--imm-accel-psd`, `--imm-sojourn`,
  `--imm-probability-floor`, `--imm-manoeuvre-on/off`.
- **Old recordings.** A header without `motion_model` is resolved to `cv`
  explicitly (`Param::if_absent` in event_log.cpp), not to the library default.
  `EventLog.OlderHeadersWithoutAMotionModelReplayWithTheLegacyFilter` and
  `Replay.RecordedLegacyRunReplaysBitForBit` cover it. The legacy filter's
  scorecard is byte-identical to the one before this change, apart from the
  echoed command.
- **Default: `imm`.** On the reported evaluation set (seeds 101-110, right
  turns 0.3-7 g at 100 and 250 m/s, disjoint from the selection sets) ID
  switches fall from 2.0-6.8 to 0.0-0.9 per run and missed instants from
  4.5-13.6% to 0-2.9% from 1 g up. Mean RMSE improves on all seven fixtures
  and in every scorecard condition. ID switches are fewer or equal in every
  condition except severe impairment (9 against 3 over 21 runs, worst target
  RMSE 55 m against 29 m, all in formation_split; docs/evaluation.md).
  The cost, kept in plain view:
  - **Dense formations.** Over the ten re-seeded copies of every multi-target
    scenario, identity is better with the IMM everywhere except
    formation_split (three targets 45 m apart). There it is worse with mild
    and moderate impairment (45 vs 26 and 42 vs 34 ID switches over 30 runs),
    about equal with severe (52 vs 54), and worse in direct feed (0.8 vs 0.3
    per run). The committed formation_split fixture shows it under severe
    impairment (8 vs 0 ID switches over three seeds). A filter that can follow
    a 5 g turn can also follow a neighbour's detections.
  - **Lost matched-model targets.** With truth drawn from the IMM's own
    switching model (up to ~3.5 g RMS white acceleration), the engine
    re-initiates 27% of 200 runs within 90 s. The mixture gate discounts a
    detection from a mode the track thinks improbable. The estimator itself is
    nearly consistent there (NEES4 4.15-4.22, NIS 2.00/2).
  - **Clutter.** The IMM's manoeuvre modes let clutter build tracks more
    easily. In the Monte Carlo clutter sweep (ADR 0015) it confirms more false
    tracks than Sifted's constant-velocity configurations and the Stone Soup
    baseline from about 5 clutter points per scan up (docs/evaluation-baseline.md).
  - **Real ADS-B prediction.** Against withheld real reports
    (docs/real-data-evaluation.md, evaluation A) the IMM predicts slightly
    worse than the legacy filter did: median 8.8 against 7.5 m at 2-5 s, and
    93.5 against 83.7 m at 40-80 s. Real paths with simulated sensors
    (evaluation B) improve: per-window median 13.8 against 15.2 m, p95 21.7
    against 36.6 m. At the live REST cadence it fragments less (2 against 8).
  - **Compute.** Three modes per track cost more estimation work. The first
    implementation also screened every measurement-track pair once per mode,
    which cut tracks per core from 2 664 (legacy) to 1 532 on the measuring
    machine. One envelope screen covering all modes
    (`estimation::may_gate_envelope`), and one prediction per track per group,
    fixed that without changing any result. Measured on one machine
    (docs/benchmarks.md, "Motion models"):
    - tracks per core: 2 898 against 2 976 for the legacy filter (97%);
    - per-scan time at 1 000-3 000 targets: +5% to +27%;
    - small pictures and the 300-target throughput workload, where estimation
      dominates: 1.6-2.7x per scan, 213 000 against 327 000 measurements/s.
- The legacy estimator stays selectable, tested (every regression, gate-screen,
  consistency and replay test runs both) and documented. It is the baseline of
  docs/motion-model-evaluation.md and the behaviour of every older recording.

## Consequences
- Straight-flight NEES4 does **not** move towards 4. It is 2.10 (legacy 2.57)
  on the manoeuvre set and 1.65 (2.01) on the fixtures. The simulator flies
  exactly straight legs, and any filter with a non-zero steady-state gain is
  pessimistic by about half against zero-acceleration truth (design.md
  section 7). The IMM's mixed-in manoeuvre modes make it slightly more so. In
  turns NEES4 falls from 32 to 2.7, and at onset from 86 to 17.
- New parameters are part of the recorded configuration. Changing a default
  changes new runs only.
- Rejected for now: spatial association aids for formations (track-to-track
  separation constraints, JPDA), which would address the formation trade-off
  but change association (ADR 0004), and a coordinated-turn mode added to the
  three CV modes.
