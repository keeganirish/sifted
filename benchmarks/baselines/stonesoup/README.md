# Stone Soup baseline

An external comparator for Sifted: a multi-target tracker assembled from
[Stone Soup](https://github.com/dstl/Stone-Soup) components and configured to
match Sifted's defaults as closely as the frameworks allow. It reads the same
scenario directories (`sensors.cfg`, `frames.hex`, `truth.csv`) and decodes the
frames with the repository's own Python decoder (`sim/sifted_sim/wire.py`). Its
tracks are written in the track report format
(`eval/sifted/eval/track_report.hpp`) and scored by Sifted's own scorer, so
both trackers are measured identically. Results:
[docs/evaluation-baseline.md](../../../docs/evaluation-baseline.md); method and
fairness rules: [ADR 0015](../../../docs/adr/0015-monte-carlo-evaluation-and-baseline.md).

This is optional developer tooling. Stone Soup is **not** a dependency of the
application, the installer, the default test suite or CI. It lives in its own
virtual environment.

## Setup and run

```bash
cmake --preset release && cmake --build --preset release      # sifted_montecarlo
python3 -m venv build/stonesoup-venv
build/stonesoup-venv/bin/pip install -r benchmarks/baselines/stonesoup/requirements.txt
build/stonesoup-venv/bin/python benchmarks/baselines/stonesoup/compare.py
```

`compare.py` writes `docs/evaluation-baseline.md` (deterministic for a given
binary, base seed and these pinned versions) and
`docs/benchmarks/baseline-timing.md` (processing time, machine dependent).
`--smoke` runs a handful of runs into `build/baseline-smoke/`. On Ubuntu,
`python3 -m venv` needs the `python3-venv` package. The environment goes under
`build/` (ignored by git), never under `benchmarks/`: installed packages ship
C++ sources that the repository's clang-format check over `benchmarks/` would
otherwise pick up.

Tests of the configuration mapping (skipped when Stone Soup is not installed;
not run by `ctest`):

```bash
build/stonesoup-venv/bin/python -m unittest discover -s benchmarks/baselines/stonesoup
```

`tracker.py <scenario_dir> <out.tracks>` tracks one scenario directory.

## Configuration mapping

Sifted's defaults are in `src/sifted/core/engine.hpp` and
`src/sifted/association/lifecycle.hpp`. The page compares Stone Soup with
three Sifted configurations: the default IMM (what Sifted ships), the legacy
constant-velocity filter with its manoeuvre process-noise switching, and
`sifted-cv-fixed-q`, a constant-velocity filter with fixed process noise. The
last is the like-for-like configuration for this baseline.

| Aspect | Sifted | Stone Soup baseline | Match |
|---|---|---|---|
| State, dynamics | [px, py, vx, vy], white-noise acceleration, q = 4 m²/s³ per axis | `CombinedLinearGaussianTransitionModel` of two `ConstantVelocity(4.0)`: same Q (tested) | exact (with `sifted-cv-fixed-q`) |
| Range-bearing update | EKF on (range, bearing), bearing = wrap(atan2(dy, dx) − yaw) | `CartesianToBearingRange` with the sensor's translation and `rotation_offset` (0, 0, yaw): same h (tested) | exact model; Stone Soup's Jacobian is numerical, Sifted's analytic |
| Bearing-only update | EKF on bearing | `Cartesian2DToBearing` with the same pose | exact model, same Jacobian caveat |
| Measurement noise | from each frame's sigma | from each frame's sigma | exact |
| Covariance update | Joseph form | `ExtendedKalmanUpdater(use_joseph_cov=True)` | exact |
| Gate | chi-square on d² at probability 0.999 (13.82 2-D, 10.83 1-D) | `DistanceHypothesiser` with `SquaredMahalanobis` and `missed_distance` = the same quantiles | exact |
| Association | global nearest neighbour (Hungarian), cost d², unassigned cost = gate, per sensor scan | `GNNWith2DAssignment` on the same hypotheses: cost d², missed = gate. Rows are tracks rather than measurements, which gives the same optimum (both minimise Σ(d² − gate)) | exact up to ties |
| Order of association | confirmed/coasting tracks first, then tentative tracks with the leftovers | main tracks first, leftovers to `MultiMeasurementInitiator`, whose holding tracks are associated by the same GNN | exact |
| Tentative tracks and bearings | tentative tracks take bearing-only updates (which do not count as hits) | holding tracks get range-bearing detections only | **differs**: Stone Soup's initiator is range-bearing only here |
| Initiation | one range-bearing fix, polar-to-Cartesian covariance, velocity 0 ± 100 m/s | `SimpleMeasurementInitiator`: inverse function and pseudo-inverse Jacobian, prior velocity 0 ± 100 m/s | same to first order |
| Confirmation | 3 position hits, tentative dropped after 3 s without one | `min_points=3` updates, `UpdateTimeDeleter(3 s)` on holding tracks | exact |
| Coasting, deletion | coasting after 2.5 s, dropped 10 s after the last position update or at predicted position σ > 1000 m (largest eigenvalue) | the same rules, applied at every scan time before association (`maintain` in `tracker.py`) | exact rules; Stone Soup has no lifecycle states of its own, so they are kept beside its tracks |
| Track cap | 1024 live tracks including tentative; extra starts counted as TrackLimit | `CappedInitiator` refuses starts at 1024 live tracks | same rule (the count is printed, not scored) |
| Manoeuvres | IMM (default) or CV with process-noise switching | none: fixed-q CV | only `sifted-cv-fixed-q` matches |
| Grouping | measurements of one sensor and timestamp form a scan | the same grouping | exact |
| Time | integer ns | `datetime` (microsecond resolution); every simulated time is a whole microsecond and is checked | exact for these inputs |
| Reported tracks | confirmed and coasting, extrapolated to each truth instant | main tracks, predicted to each truth instant | exact |

**Not used for tracking:** truth positions. `truth.csv` supplies only the
reporting instants, as for Sifted (tested by scrambling truth positions).

**Tuning:** none, for either tracker. Both use the configuration above,
derived from Sifted's documented defaults. No parameter was changed after
looking at results. So there is no tuning set, and every reported run is
effectively held out.

## Run counts

Stone Soup's per-scan cost grows with (tentative tracks × detections), each
pair evaluated through Python objects. One run takes about 1 s at the default
clutter, about 10 s at 20 clutter points per scan, 40–70 s at 50 and 120–200 s
at 100 (measured on the development machine). `compare.py` therefore uses 20
runs per scenario (100 per condition) for the noise set and clutter up to 20,
4 per scenario (20 per condition) at clutter 50 and 2 per scenario (10) at
clutter 100. These are the first runs of each scenario in the Sifted-only
Monte Carlo sets, with the same seeds and the same files. The
heavy-clutter intervals are correspondingly wider. They are reported as
computed, and the page states the run count of every condition. `--full` uses
20 per scenario everywhere (several hours).

## Where the frameworks cannot be matched exactly

- Bearing-only updates of tentative tracks (above).
- Numerical (Stone Soup) and analytic (Sifted) measurement Jacobians differ in
  the last digits.
- Stone Soup appends a prediction to a track at every scan it misses. Sifted
  predicts from the last update. For this white-noise acceleration model the
  two give the same prediction (the discretised Q composes exactly), up to
  rounding.
- Tie-breaking among equal-cost assignments may differ. Inputs are given to
  Stone Soup as `OrderedSet`s so its own choice is deterministic.
- Processing time: compiled C++ against interpreted Python on numpy and a
  general-purpose framework. See `docs/benchmarks/baseline-timing.md` for the
  measurement and its limits.
