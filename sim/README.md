# sim

Sensor simulators and scenario generators (Python, standard library only for now).

| Module | What it does |
|---|---|
| `wire` | Python mirror of the C++ wire codec, checked against `tests/fixtures/` |
| `trajectory` | closed-form ground truth: straight segments and coordinated turns |
| `radar` | range-bearing radar with configurable pose, noise, rate and seed |
| `sensors` | sensor table writer/reader (`sifted-sensors 1`, ADR 0003) |
| `scenarios` | named, reproducible scenarios (`straight_crossing`, `turn`) |
| `generate` | CLI that writes a scenario's `sensors.cfg`, `frames.hex`, `truth.csv` |

Determinism: noise uses `random.Random(seed).random()` (stable across Python
versions) with our own Box-Muller transform, and timestamps are integer
nanoseconds. Same scenario, same machine: identical bytes. Across platforms the
last bit of a float can differ (libm `sin`/`cos`/`atan2`), so the fixture test
compares decoded values with a tight tolerance.

```bash
cd sim
PYTHONPATH=. python3 -m unittest discover -s tests          # tests
python3 -m sifted_sim.generate --list                        # scenario names
python3 -m sifted_sim.generate turn ../tests/fixtures/scenarios/turn
```

Planned (M2): camera-like (bearing-only) sensor, multiple targets, failure modes.
