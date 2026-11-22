# sim

Sensor simulators and scenario generators (Python, standard library only: no third-party dependencies).

| Module | What it does |
|---|---|
| `wire` | Python mirror of the C++ wire codec (encoder and strict decoder), checked against `tests/fixtures/` |
| `trajectory` | closed-form ground truth: straight segments and coordinated turns |
| `radar` | range-bearing radar with configurable pose, noise, rate and seed; multi-target scans with coverage, detection probability, clutter and dropouts |
| `camera` | bearing-only sensor with pose, noise, rate, phase, field of view, detection probability and dropouts |
| `sensors` | sensor table writer/reader (`sifted-sensors 1`, ADR 0003) |
| `scenarios` | named, reproducible scenarios: single-target `straight_crossing`, `turn`; multi-target `crossing_pair`, `close_pass`, `enter_leave`, `formation_split` (GNN vs greedy, M3), `demo` (docs/demo.md) |
| `generate` | CLI that writes a scenario's `sensors.cfg`, `frames.hex`, `truth.csv` |
| `manoeuvres` | writes Monte Carlo sets of the parametric `manoeuvre(load_g, speed, seed)` scenario (evaluation and tuning sets) and re-seeded copies of the fixtures, into the build directory, for `sifted_motion_eval` (ADR 0014) |
| `montecarlo` | Monte Carlo sets for `sifted_montecarlo` (ADR 0015): independent noise realisations of every multi-target scenario and a clutter x Pd sweep, seeds derived by SplitMix64 from (base seed, scenario, run), into the build directory; `report` regenerates docs/evaluation-monte-carlo.md |
| `send` | sends a scenario's frames to `sifted_fusiond` over UDP, real time or accelerated, optionally with injected sensor faults (`--fault`) |
| `impair` | the degraded-network model (ADR 0008; exact mirror of `src/sifted/net/impairment.hpp`) and a live UDP impairment proxy |
| `faults` | seeded sensor faults: dropout, intermittent, restart, clock offset/drift, bias, stuck (M3) |
| `adsb` | OpenSky Network adapter: state vectors -> GeoPosition frames over UDP (ADR 0007) |
| `geo` | WGS-84 geodetic <-> local ENU plane, mirroring the C++ projection; ECEF chord distances |
| `statevectors` | streaming, validating reader for OpenSky historical state-vector CSV, plain or inside `.tar.gz` (untrusted input, threat model B6) |
| `realclean` | cleaning real reports into flights (ground, duplicates, stale repeats, impossible jumps, gap splits, short segments, shared addresses), every removal counted |
| `recorded` | `RecordedTrajectory`: a real flight path with the `Trajectory` interface (Hermite or linear interpolation), and leave-one-out error |
| `realeval` | the real-data evaluation: `fetch`, `run`, `describe` (docs/real-data-evaluation.md) |

Determinism: noise uses `random.Random(seed).random()` (stable across Python
versions) with our own Box-Muller transform, and timestamps are integer
nanoseconds. Failure modes (detection draws, clutter) use a second seeded
stream, and noise is drawn for every target whether detected or not, so
changing a failure mode never changes the noise of the remaining detections. Same scenario, same machine: identical bytes. Across platforms the
last bit of a float can differ (libm `sin`/`cos`/`atan2`), so the fixture test
compares decoded values with a tight tolerance.

```bash
cd sim
PYTHONPATH=. python3 -m unittest discover -s tests          # tests
python3 -m sifted_sim.generate --list                        # scenario names
python3 -m sifted_sim.generate turn ../tests/fixtures/scenarios/turn
python3 -m sifted_sim.send crossing_pair --target 127.0.0.1:5600 --speed 5
python3 -m sifted_sim.adsb --from-file tests/samples/opensky_states_sample.json --print
# live impairment between sender and daemon (ADR 0008):
python3 -m sifted_sim.impair --listen 127.0.0.1:5601 --target 127.0.0.1:5600 \
    --seed 7 --loss 0.03 --burst 0.01 0.25 0.6 --jitter-ms 400 --duplicate 0.02 --outage 30:38
python3 -m sifted_sim.send crossing_pair --target 127.0.0.1:5601 --fault clock:20:+2
```

`tests/test_adsb_fuzz.py` property-tests the OpenSky adapter with random and
mutated JSON (standard library only); `tests/test_statevectors_fuzz.py` does
the same for the state-vector CSV reader. `tests/test_realeval.py` covers the
real-data pipeline on synthetic inputs; its `EndToEnd` class runs the C++
tools when `SIFTED_BIN_DIR` names their directory:

```bash
SIFTED_BIN_DIR=../build/release/eval PYTHONPATH=. python3 -m unittest tests.test_realeval
python3 -m sifted_sim.realeval run    # real data: docs/development.md, "Real-data evaluation"
```

`tests/samples/opensky_states_sample.json` is a hand-written response in the
format documented by OpenSky (not a captured one; no network access was
available when it was written).
