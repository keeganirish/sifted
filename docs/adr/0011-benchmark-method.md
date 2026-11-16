# ADR 0011: Benchmark method -- own harness, seeded workloads, distributions, profile before optimising

Status: accepted (M3)

## Context
M3 must report latency, throughput and tracks handled per core honestly and
reproducibly, and optimise only what a profile shows.

## Decision
- **Own harness, no new dependency** (`benchmarks/bench_engine.cpp`). Google
  Benchmark is apt-packaged but reports means of repeated runs of a fixed
  body; we need per-scan latency *distributions* (median, p95, p99, max) of a
  stateful engine whose cost depends on the history (live tracks), which a
  plain loop with `steady_clock` around `Engine::on_scan` gives directly.
  Timing code lives only in the harness.
- **Seeded synthetic workloads** (`benchmarks/workload.hpp`): one radar,
  N straight-flying targets in a 50 km disc, p_detect 0.95, the simulator's
  noise, C uniform clutter points per scan; 10 warm-up scans so tracks are
  confirmed, then timed scans. Same seed, same workload everywhere.
- **Variance**: every configuration runs R times (default 5); the report
  gives the median over runs of each statistic and the min..max spread of
  the per-run medians.
- **Tracks per core**: the largest N (bisection, ~5% resolution, 10%
  clutter) for which one engine thread's p99 scan time stays within a stated
  budget (100 ms) at a stated update rate (one scan of every target per
  second).
- **End to end**: `benchmarks/e2e_latency.py` drives the real
  `sifted_fusiond` over loopback UDP and reads its WebSocket stream.
- **Release preset only**; machine, compiler and run count stated in
  docs/benchmarks.md; never report an unmeasured number.
- **Profile first** (callgrind on a RelWithDebInfo build): 70%+ of engine
  time at 1 000 targets was the all-pairs gating computation (atan2,
  Jacobian, `H P H'`, Cholesky per measurement-track pair). The one
  optimisation made is the exact gate screen (ADR 0004): before/after numbers
  in docs/benchmarks.md; every regression and replay test re-run unchanged.

## Consequences
+ Numbers are reproducible with one command per table.
- Absolute numbers are machine-specific; the report says which machine.
