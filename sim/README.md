# sim

Sensor simulators and scenario generators (Python, standard library only for now).

| Module | What it does |
|---|---|
| `wire` | Python mirror of the C++ wire codec, checked against `tests/fixtures/` |
| `trajectory` | closed-form ground truth: straight segments and coordinated turns |
| `radar` | range-bearing radar with configurable pose, noise, rate and seed; multi-target scans with coverage, detection probability, clutter and dropouts |
| `camera` | bearing-only sensor with pose, noise, rate, phase, field of view, detection probability and dropouts |
| `sensors` | sensor table writer/reader (`sifted-sensors 1`, ADR 0003) |
| `scenarios` | named, reproducible scenarios: single-target `straight_crossing`, `turn`; multi-target `crossing_pair`, `close_pass`, `enter_leave` |
| `generate` | CLI that writes a scenario's `sensors.cfg`, `frames.hex`, `truth.csv` |
| `send` | sends a scenario's frames to `sifted_fusiond` over UDP, real time or accelerated |
| `adsb` | OpenSky Network adapter: state vectors -> GeoPosition frames over UDP (ADR 0007) |

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
```

`tests/samples/opensky_states_sample.json` is a hand-written response in the
format documented by OpenSky (not a captured one; no network access was
available when it was written).
