# ADR 0003: Sensor poses come from a static sensor table, not the wire

Status: accepted (M1)

## Context
A `Measurement` carries a `sensor_id`, but a range-bearing measurement only
means something once you know where the sensor stood and which way it pointed:
the EKF's `h(x)` and Jacobian need the sensor's position and boresight yaw in
the tracking frame. Something has to map `sensor_id` to a pose, and that
mapping has to be identical during a live run and its replay (ADR 0002), since
a different pose produces different tracks from the same frames.

## Decision
Sensor poses are **operator configuration**: a `SensorTable`
(`src/sifted/core/sensor.hpp`) that maps each `SensorId` to its kind and
`SensorPose {x_m, y_m, yaw_rad}` in the local tracking frame (x east, y north,
angles counter-clockwise from +x; a bearing is measured from the boresight).
The `Engine` is constructed with a table and looks every measurement's sensor
up in it; nothing in the EKF or the engine hard-codes a sensor.

The table has a small canonical text form, `sifted-sensors 1`:

```
sifted-sensors 1
sensor <id> <kind> <x_m> <y_m> <yaw_rad>
```

Parsing is strict (exact token counts, whole-token numbers, unique ids, range
checks, 64 KiB and 1024-sensor caps). Doubles are written in shortest
round-trip form (`repr()` in Python, `std::to_chars` in C++) and read with
`std::from_chars`, so a pose survives a write/read cycle **bit-exactly**.
`format_sensor_table()` produces one canonical text per table (sorted by id).

The simulator (`sim/sifted_sim/sensors.py`) writes this format next to the
frames it generates, the regression test reads it, and `sifted_fusiond` will
load it at start-up (M2).

**Replay:** when the event log lands (M2), its header will embed the canonical
table text verbatim. The replayer rebuilds the `SensorTable` from that header,
not from whatever config file is on disk at replay time, so a recording is
self-contained and replays to the same bits.

## Alternatives considered
- **Pose in every measurement frame.** Self-describing, but it changes the v1
  wire format (and ADR 0001's canonical layout), adds 24 bytes to every
  datagram, and, worst, lets any host that can reach the UDP port (boundary B1,
  untrusted) move a sensor by lying about its pose.
- **A new `SensorConfig` wire message.** No v1 change for measurements, but the
  same trust problem, plus ordering questions (measurement before config) and
  state that the network can change mid-run.
- **Compile-time or test-only constants.** Simplest, but not reusable by the
  simulator, daemon and replayer, and it hard-codes scenarios into the core.
- **JSON/TOML config.** Familiar, but needs a parser dependency or a much larger
  hand-written one, and makes bit-exact float round-trips harder to guarantee.

## Consequences
+ Wire format v1 is unchanged; the network cannot alter sensor geometry.
+ One table type serves the simulator, engine, regression tests and replay.
+ Bit-exact text round-trip keeps replay deterministic and fixtures diffable.
- Poses are static for a run. Moving sensors (or ADS-B, whose positions are
  geodetic, not sensor-relative) need more: a geodetic reference point for the
  tangent plane is still an open question in `docs/design.md`.
- A second parser now exists. It reads trusted operator input (new boundary B5
  in the threat model), but it is still strict and size-limited; a fuzz target
  can be added cheaply if it ever reads anything less trusted.
- Changing the text format needs a version bump (`sifted-sensors 2`), exactly
  like the wire format.
