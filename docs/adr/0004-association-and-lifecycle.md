# ADR 0004: Chi-square gating, two-pass global nearest neighbour, measurement-time lifecycle

Status: accepted (M2). *Recorded retrospectively in M3. The M2 code and
comments referenced this decision before it was written down; this record
documents the decisions that implementation embodies. M3 added the greedy comparison method and the gate
screen (sections below marked M3).* ADR 0014 adds the IMM motion model: for
IMM tracks the gating statistic and assignment cost is the mixture statistic
over the modes (the same chi-square gate and solver), and manoeuvre handling
is the IMM's mode probabilities instead of NIS process-noise switching, which
remains for the legacy `cv` model.

## Context
With many targets and several sensors, every measurement must be attributed
to one track, to a new track, or to nothing, deterministically and in a way
that can be explained afterwards. Sensors measure in different spaces
(range-bearing, bearing-only, geodetic position) with different noise.

## Decision
- **Gating** in each sensor's own measurement space: the squared Mahalanobis
  distance `d2 = y' S^-1 y`, `S = H P H' + R`, against the chi-square quantile
  for the measurement dimension at `gate_probability = 0.999` (10.83 for 1-D,
  13.82 for 2-D). Pairs within `near_miss_factor` (4) x gate are traced as
  near misses.
- **Assignment** by global nearest neighbour: an exact minimum-cost
  assignment (Hungarian with potentials) per connected cluster of the
  candidate graph, with a fixed cost for leaving a measurement unassigned.
  Two passes: confirmed and coasting tracks first, then tentative tracks with
  what is left, so a young track cannot steal an established track's
  measurement. Ties break by track id (columns ordered by id; the solver keeps
  the lowest index on equal cost).
- **Grouping:** consecutive measurements with the same sensor and timestamp
  (a radar scan) are associated jointly; anything else is a group of one.
- **Lifecycle** tentative -> confirmed (3 position-observing hits) ->
  coasting (2.5 s without an update) -> dropped (10 s; tentative after 3 s; or
  predicted position sigma above 1 km). Timers run on measurement time, never
  the wall clock. Bearing-only updates refine a track but never confirm it or
  keep it alive: one bearing cannot fix range.
- **Initiation** only from measurements that observe a full position
  (range-bearing, geodetic), under a hard live-track cap (`max_tracks`).
- **(M3) Greedy assignment** (`AssociationConfig::method = Greedy`) exists for
  comparison only: tests and benchmarks use it to show what the global solve
  buys (`MultiTargetRegression.FormationSplit`). Recorded in the event log
  header (`param assignment`).
- **(M3) Gate screen.** A conservative bound (`estimation::may_gate`, proof in
  its header and `docs/design.md` section 10) skips the full gating
  computation for pairs that provably cannot be within `near_miss_factor x
  gate`. It never changes a result -- verified by `GateScreen.*` tests and by
  the engine fuzz target, which runs a second engine with the screen off --
  so it is not recorded in logs.

## Alternatives considered
- Greedy nearest neighbour: simpler, but wrong under ambiguity
  (formation_split shows ID switches and ~3x RMSE).
- JPDA / MHT: better in dense clutter, much more complex and harder to make
  explainable and deterministic. Out of scope.

## Consequences
+ Deterministic, explainable decisions: every association, competition and
  near miss is in the per-track trace.
- Cost per group grows with measurements x tracks (the screen makes the
  per-pair cost small, but it is still all pairs; see docs/benchmarks.md).
- Hard ambiguity (targets closer than ~3 sigma) can still swap identities;
  an optimal assignment is not infallible.
