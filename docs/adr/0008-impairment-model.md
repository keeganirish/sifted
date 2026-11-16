# ADR 0008: A seeded, per-link degraded-network model shared by C++ tests and a live Python proxy

Status: accepted (M3)

## Context
M3 must show how the track picture degrades and recovers under loss, delay,
jitter, reordering, duplication and outages, deterministically in tests and
live in front of the daemon, with the same model in both places.

## Decision
One model, two implementations that produce identical output:
`src/sifted/net/impairment.{hpp,cpp}` (C++, used offline by tests) and
`sim/sifted_sim/impair.py` (Python, offline and as a live UDP proxy,
`python3 -m sifted_sim.impair`). Per frame, on its sensor's link:

1. outage: sent inside a scheduled `[start, end)` window (per link, or for
   every link) -> lost;
2. loss: a two-state Gilbert-Elliott chain steps once per frame (Good -> Bad
   with `p_good_to_bad`, Bad -> Good with `p_bad_to_good`), then the frame is
   lost with probability `loss` (Good) or `loss_bad` (Bad); independent loss
   is the special case `p_good_to_bad = 0`;
3. delay: `delay_ns + U{0..jitter_ns}`; reordering is a *consequence* of
   jitter exceeding the gap between frames, not a separate knob;
4. duplication: with probability `duplicate`, a second copy with its own
   independently drawn delay.

Profiles are global with per-sensor overrides. Arrivals are sorted by
(arrival time, source index, duplicate flag).

**Determinism.** Each link has its own SplitMix64 stream seeded from
`(seed, sensor id)`, and every frame consumes exactly five draws whatever
happens to it. So a sensor's fate depends only on the seed, its profile and
its own frame sequence -- not on other sensors' traffic or on earlier
outcomes (tested: `Impairment.LinksAreIndependentOfOtherSensorsTraffic`).
Only integer arithmetic and exact IEEE comparisons are used (uniforms are
53-bit fractions, bounded integers use a portable 64x64->128 multiply), so
C++ and Python agree exactly; a committed golden fixture
(`tests/fixtures/impairment/golden.txt`, written by
`python3 -m sifted_sim.impair --write-golden`) is checked by both test suites.

**Live proxy.** The proxy stamps arrivals with its monotonic clock (zero at
start; outage windows are relative to it), asks the same `Link` for each
datagram's fate and forwards copies at their arrival times. Its pacing is
subject to OS scheduling; what matters for reproducibility is the daemon's
event log, which records what actually arrived (ADR 0005). The proxy holds at
most 65536 datagrams (counted overflow beyond that).

## Alternatives considered
- `tc netem`: realistic, but needs root, is Linux-only, and is not
  reproducible frame by frame; cannot be used in unit tests.
- Independent loss only: misses the bursty losses real links show, which are
  what turns into coasting and track breaks.
- A separate "reorder probability": unphysical; reordering on real networks
  comes from variable delay.

## Consequences
+ Degraded-network tests are fully deterministic and run in milliseconds,
  through the real `net::Pipeline` (no sockets).
+ The same impairment can be shown live in the demo.
- Delay is uniform, not heavy-tailed; a Pareto tail would need `log()`, whose
  last bits differ across libms and would break C++/Python equality.
