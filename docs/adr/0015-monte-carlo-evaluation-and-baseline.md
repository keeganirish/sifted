# ADR 0015: Monte Carlo evaluation, GOSPA, and a fair external baseline

Status: accepted

## Context
The scorecard (`docs/evaluation.md`) scores one sensor-noise realisation per
scenario. Its seeds (7, 8, 9) vary only the network. It reported 0 ID switches,
0 false tracks and a worst RMSE of 12.20 m over seven fixtures, and every
number was correct. An independent stress test of `df9666e` with fresh noise
seeds found 19 ID switches in 100 runs of the five multi-target scenarios (17
in `formation_split`), a worst target RMSE of 56 m, and false tracks that grow
from a few to hundreds per run as clutter rises from 20 to 100 points per scan.
A single realisation cannot show uncertainty, and with no comparator a reader
cannot tell whether any number is good.

## Decision

### Monte Carlo sets, seeds and the unit of analysis
`sim/sifted_sim/montecarlo.py` writes independent realisations of every
multi-target scenario (the *noise* set, 100 runs each) and a clutter × Pd
sweep (clutter 0.5, 5, 20, 50, 100 points per scan at Pd 0.9 and 0.7 on every
radar, 20 runs per scenario = 100 per condition) into `build/montecarlo/`.
They are never committed. Seeds are derived, never chosen:
`run_seed = mix(mix(mix(base) ^ fnv(scenario)) ^ mix(run))`, then one per
sensor, with `mix` = one SplitMix64 output. Full-avalanche mixing gives
unrelated 64-bit seeds for neighbouring runs and sensors, avoiding the
correlated streams of consecutive small Mersenne Twister seeds. Tests check
that no two streams coincide and that first draws of neighbouring runs are
uncorrelated. The seed depends on (scenario, run) only. Every condition and
every tracker sees the same noise in run r (common random numbers), so all
comparisons are paired.

The **run** is the independent unit. Every interval is computed from one value
per run, never from time samples within a run, which share the filter's
memory. Statistics (`eval/sifted/eval/stats.hpp`):
- means with a seeded **percentile bootstrap** (2000 resamples). Per-run
  counts (ID switches, false tracks) are skewed and often zero, so a normal
  interval is wrong; the bootstrap assumes no shape;
- with **zero events** in n runs, the exact Poisson bound 0 [0, 3.69/n]
  instead of the bootstrap's degenerate [0, 0];
- **fractions of runs** with any identity break or false track, with the
  Wilson interval, so that a few bad runs cannot hide in an average;
- **paired differences** (bootstrap of the mean per-run difference) for
  network vs direct, Pd 0.7 vs 0.9, and tracker vs tracker;
- **convergence**: interval width against run count, published with the
  results to justify the run count.

100 runs per condition keeps every mean's 95% interval within a few percent
of the quantities compared (the convergence table shows it). The whole
Sifted-only evaluation takes about 4 minutes.

### GOSPA
`eval/sifted/eval/gospa.hpp`: GOSPA with alpha = 2 (Rahmathullah,
García-Fernández and Svensson, FUSION 2017, Definition 1 and Proposition 1,
checked against the paper). With alpha = 2 the metric is the minimum over
partial assignments of Σ d^p plus c^p/2 per missed target and per false track.
That is `association::solve_assignment` with pair cost d^p for d < c,
forbidden pairs beyond c, and c^p as the unassigned-row cost: the objective
differs only by a constant. Defaults are c = 100 m (the scorer's match
distance, so a GOSPA miss is what CLEAR-MOT calls a miss) and p = 2 (RMSE-like).
Both are parameters and are printed with the results. It is computed per truth
instant over the reported tracks against the targets eligible for "missed".
Tracks matched to present-but-unscored targets (out of coverage, or in their
burn-in) are "don't care". GOSPA is reported as an RMS over instants with its
exactly additive localisation / missed / false decomposition. The CLEAR-MOT
metrics are unchanged.

### Tracker-independent scoring
`Scorer::score` takes reported tracks from a live `Engine` or as a list.
Metrics that need engine internals (NIS, the engine's counters such as
TrackLimit) are flagged unavailable (`Score::engine_metrics`), never reported
as 0. The list comes from a **track report** file
(`eval/sifted/eval/track_report.hpp`): `sifted-tracks 1`, one row per
(truth instant, track) with time, id, status, last update, state and an
optional covariance triangle. The file is untrusted input. It is parsed
strictly, with size, line and row limits, finite bounded numbers, sorted
unique rows and a positive-definite covariance, and fuzzed
(`fuzz_track_report`). Scoring adds the checks that need the truth: rows only
at truth instants, and no reuse of an ended track id. Neutrality is proven two
ways. `docs/evaluation.md` regenerates byte-identically, and Sifted's tracks
exported to a file score exactly like in-process scoring (`Eval.FileScoring*`).
The exported covariance mirrors the lower triangle, which is the one the NEES
Cholesky reads, so the match is bit for bit.

### Baseline and fairness rules
`benchmarks/baselines/stonesoup/` (Stone Soup 1.9.1, every package pinned,
its own virtual environment; not a dependency of the app, installer, default
tests or CI). The rules:
1. **Same inputs**: the same scenario files, decoded with the repository's
   decoder. Truth is used only for the reporting instants.
2. **Same scorer**: the baseline writes track reports; `sifted_montecarlo`
   scores them and Sifted with one Scorer.
3. **Same models and settings**: measurement noise from the frames, the same
   process-noise density, the same chi-square gate, GNN, and the same
   initiation, confirmation, coasting and deletion rules. Every mapping and
   every mismatch is listed in the baseline README. The like-for-like Sifted
   configuration (`sifted-cv-fixed-q`) is reported beside Sifted's defaults.
4. **No asymmetric tuning**: neither tracker was tuned. Any future tuning must
   use a separate tuning set, the same effort for both, and report held-out
   seeds.
5. **Publish losses**: conditions where the baseline is better are reported
   as such. The comparison is not adjusted after seeing results.
6. **Time with caveats**: processing time is on a separate page, because it is
   machine dependent, and it states the language and framework differences
   that limit it.

### Pages
`docs/evaluation-monte-carlo.md` (`python3 -m sifted_sim.montecarlo report`)
and `docs/evaluation-baseline.md` (`benchmarks/baselines/stonesoup/compare.py`)
are generated, deterministic for a given binary, base seed and pinned baseline
version, and contain no timestamps or machine paths. `docs/evaluation.md`
stays the fast single-realisation regression scorecard and links to both.

## Consequences
- Headline claims now carry intervals and the fraction of runs with identity
  breaks. Weaknesses that were invisible in one realisation
  (`formation_split` identity breaks, false-track growth with clutter) are
  measured and published, not fixed here.
- Determinism holds for the same binary and architecture. Another compiler or
  maths library may change individual runs' last bits (simulated noise uses
  libm). The intervals describe exactly that run-to-run variation.
- Stone Soup's run time limits the baseline's heavy-clutter run counts. They
  are stated on the page.
