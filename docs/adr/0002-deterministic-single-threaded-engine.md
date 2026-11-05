# ADR 0002: Deterministic, single-threaded fusion engine

Status: accepted (M0)

## Context
The headline feature is bit-for-bit replay. Parallelism inside the estimator
would make results depend on scheduling.

## Decision
The `Engine` is single-threaded and a pure function of its ordered input and a
simulated clock. Concurrency exists only at the edges (network receive, track
publication), joined to the engine by a bounded ordered queue. See
`docs/design.md` §3 for the rules this implies.

## Consequences
+ Replay equality is testable with a byte comparison of output.
+ Race analysis is confined to a few well-defined handoff points.
- Throughput per engine is bounded by one core. Acceptable: the benchmark
  metric is "tracks handled per core", and sharding by spatial region is a
  possible later extension if it can preserve determinism.
