# association

Measurement-to-track assignment and the track lifecycle (ADR 0004). Implemented in M2.

- `gating.hpp`: chi-square quantiles and CDF (dof 1-100) for gates sized by measurement dimension and a gate probability.
- `assignment.hpp`: exact minimum-cost assignment with "unassigned" (Hungarian, ties to the lowest column = lowest track id) and connected-component clustering of the candidate graph.
- `lifecycle.hpp`: `TrackStatus` (tentative, confirmed, coasting, dropped), `DropReason` and `LifecycleConfig` with the documented defaults.

The engine (`core/engine.cpp`) drives these: gate in each sensor's measurement space, two-pass GNN (established tracks first), then lifecycle at measurement time. Parameter reasoning: `docs/design.md` section 6.
