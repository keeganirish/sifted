# ADR 0009: A bounded, deterministic reorder stage in ingest (0.5 s in the daemon)

Status: accepted (M3). Resolves design.md open question 2 ("out-of-order
measurements: drop, or retrodict").

## Context
Since M1 the engine drops any measurement older than its clock (OutOfOrder,
counted). With delay jitter, frames overtake each other: a camera frame
stamped t+0.25 s that arrives before the radar scan stamped t makes the whole
scan OutOfOrder, and jitter inside a scan splits it into several groups that
are associated separately. The question was whether to keep dropping, or
repair the order before the engine, and how much latency that may cost.

## Measurements
The degraded-network sweep (`DegradedNetwork.DISABLED_Sweep`; levels in
`tests/degraded_network_test.cpp`: mild = 1% loss, 20+0..50 ms delay;
moderate = 3% + bursts, 50+0..400 ms; severe = 8% + bursts, 100+0..800 ms;
all with duplication) over crossing_pair, close_pass and enter_leave, three
impairment seeds each (9 runs per cell). RMSE is what the operator sees at
each truth instant, so holding data back shows up as error.

| level | window s | out-of-order % | missed % | mean RMSE m | worst RMSE m | ID switches |
|---|---:|---:|---:|---:|---:|---:|
| clean | 0 | 0.0 | 0.00 | 11.10 | 18.69 | 0 |
| clean | 0.5 | 0.0 | 0.00 | 11.45 | 19.18 | 0 |
| clean | 1.0 | 0.0 | 0.23 | 13.99 | 27.73 | 0 |
| moderate | 0 | 10.1 | 0.43 | 12.64 | 21.64 | 1 |
| moderate | 0.5 | 0.0 | 0.32 | 12.12 | 20.81 | 1 |
| severe | 0 | 29.0 | 6.52 | 15.90 | 29.85 | 6 |
| severe | 0.25 | 6.5 | 0.68 | 13.88 | 27.14 | 1 |
| severe | 0.5 | 0.1 | 0.52 | 13.87 | 29.37 | 1 |
| severe | 1.0 | 0.0 | 0.96 | 15.54 | 32.06 | 1 |

(Full table, including 2 s and formation_split: docs/design.md section 9.)

## Decision
Add a **reorder stage** to `net::Pipeline`, between duplicate suppression
and scan assembly (`PipelineConfig::reorder_window_s`, `reorder_capacity`,
both recorded in the event log header):

- accepted frames are held, sorted by (measurement time, sensor, arrival
  order);
- at receive time `now`, let W be the latest measurement time among held
  frames received at or before `now - window`; every held frame with
  measurement time <= W is released, in order. Released times never
  decrease. Frames, quiet ticks and `finish()` release (`finish()` releases
  everything), so the daemon's existing 100 ms quiet tick drives it;
- a frame older than what was already released ("late") skips the stage --
  holding it cannot help -- and the engine judges it (usually OutOfOrder);
- at most `reorder_capacity` (4096) frames are held; beyond that the oldest
  is released early (counted).

The stage sees only receive times, which the event log records, so replay
stays bit for bit (`Reorder.ImpairedRecordedRunReplaysBitForBit`, and a live
daemon run through the impairment proxy, docs/design.md section 9). The
engine itself is unchanged: it still drops what is older than its clock.

**Window: 0.5 s in the daemon** (`--reorder-window`), **0 in the library
default** (`PipelineConfig{}`): offline feeds and M2 recordings are already
in order and keep the exact M2 behaviour and output. 0.5 s recovers
essentially all the data at the severe level (29% -> 0.1% out of order,
missed target-instants 6.5% -> 0.5%, ID switches 6 -> 1) for +0.35 m mean
RMSE on a clean network. 0.25 s is not enough at severe; 1 s and above cost
far more on clean networks (+26% RMSE, missed instants appear) and recover
nothing more.

## Alternatives considered
- **Keep dropping**: loses 10-29% of the data at moderate to severe jitter.
- **Retrodiction / out-of-sequence measurement filtering** (Bar-Shalom's
  OOSM algorithms): no added latency, but every track must keep state history
  and re-run association backwards; much more complex, and association
  decisions already made cannot be revisited cleanly. The bounded stage
  gives nearly all the benefit for a fixed, small latency.

## Consequences
+ Jitter up to ~the window costs almost nothing; scans split by jitter are
  reassembled and associated jointly again.
- Up to window + 100 ms of added latency in the live picture (end-to-end
  numbers in docs/benchmarks.md).
- Data later than the window is still dropped (counted as late and
  OutOfOrder).
