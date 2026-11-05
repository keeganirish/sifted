# sim

Sensor simulators and scenario generators (Python, standard library only for now).

Today: `sifted_sim.wire`, the Python mirror of the C++ wire codec, checked against
`tests/fixtures/`. Planned (M1): radar-like and camera-like sensors with noise,
update rate and failure modes, driven by ground-truth trajectories that are also
saved for regression scoring.

```bash
cd sim && PYTHONPATH=. python3 -m unittest discover -s tests
```
