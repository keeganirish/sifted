# ADR 0007: Fixed tangent-plane origin per run, recorded in the log

Status: accepted (M2). *Recorded retrospectively in M3 from the M2
implementation, which referenced it. Resolves design.md open question 1.*

## Context
The tracker works in a flat local x/y frame. Geodetic inputs (ADS-B) must be
projected into it, and the map needs the inverse. The projection must be
identical in a live run and its replay.

## Decision
The East-North-Up tangent plane at a **fixed origin per run**, given by the
operator (`--origin lat,lon[,alt]`; default near Greenwich for simulated
scenarios), recorded in the event log header. Not "first fix": that would
make the frame depend on which aircraft happened to be heard first. Points
more than `kMaxTangentPlaneRangeM` from the origin are refused
(`OutOfFrame`): the flat-plane approximation degrades with distance. Without
an origin, geodetic measurements are rejected as `UnsupportedKind`.

## Consequences
+ Replay projects exactly as the live run did.
- The operator must choose an origin inside the area of interest; documented
  in the ADS-B section of docs/development.md.
