"""Writes a scenario's sensor table, encoded frames and ground truth to a directory.

    python3 -m sifted_sim.generate <scenario> <out_dir>
    python3 -m sifted_sim.generate --list

Outputs (consumed by tests/scenario_regression_test.cpp and
tests/multi_target_regression_test.cpp):

    sensors.cfg   sensor table, "sifted-sensors 1" text format (ADR 0003)
    frames.hex    one wire-v1 frame per line, hex-encoded, in send order
    truth.csv     ground truth, in one of two layouts told apart by the header:

      time_ns,x_m,y_m,vx_mps,vy_mps            single target (M1): one row
                                               per frame, same order
      time_ns,x_m,y_m,vx_mps,vy_mps,target_id,covered
                                               multi-target (M2): one row per
                                               (truth time, target) for every
                                               target that exists, sorted by
                                               time then target id; covered=1
                                               if it is inside the coverage of
                                               a sensor that can start tracks

The second layout only appends columns, so readers of the first keep
working on M1 fixtures. Frames of a multi-sensor scenario are merged in time
order (ties: sensor id, then each sensor's own order). Floats are written
with repr() (shortest round-trip form).
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

from . import camera, radar, scenarios, sensors, wire

TRUTH_HEADER_V1 = "time_ns,x_m,y_m,vx_mps,vy_mps"
TRUTH_HEADER_V2 = TRUTH_HEADER_V1 + ",target_id,covered"
NS_PER_S = radar.NS_PER_S


def merged_detections(s: scenarios.MultiScenario) -> list[radar.Detection]:
    """Every sensor's detections, merged into one send order (stable)."""
    per_sensor: list[list[radar.Detection]] = []
    for r in s.radars:
        per_sensor.append(radar.observe_targets(r, s.targets, s.duration_s, s.start_ns))
    for c in s.cameras:
        per_sensor.append(camera.observe_targets(c, s.targets, s.duration_s, s.start_ns))
    tagged = [(d.measurement.time_ns, d.measurement.sensor_id, i, d)
              for dets in per_sensor for i, d in enumerate(dets)]
    tagged.sort(key=lambda x: (x[0], x[1], x[2]))
    return [x[3] for x in tagged]


def truth_rows(s: scenarios.MultiScenario) -> list[str]:
    rows = [TRUTH_HEADER_V2]
    period_ns = round(NS_PER_S / s.truth_rate_hz)
    for offset_ns in range(0, int(s.duration_s * NS_PER_S) + 1, period_ns):
        t_s = offset_ns / NS_PER_S
        for tgt in sorted(s.targets, key=lambda x: x.target_id):
            st = tgt.state_at(t_s)
            if st is None:
                continue
            covered = any(math.hypot(st.x_m - r.x_m, st.y_m - r.y_m) <= r.max_range_m
                          for r in s.radars)
            rows.append(f"{s.start_ns + offset_ns},{st.x_m!r},{st.y_m!r},{st.vx_mps!r},"
                        f"{st.vy_mps!r},{tgt.target_id},{int(covered)}")
    return rows


def render_multi(s: scenarios.MultiScenario) -> dict[str, str]:
    dets = merged_detections(s)
    configs = [r.config for r in s.radars] + [c.config for c in s.cameras]
    return {
        "sensors.cfg": sensors.format_table(configs),
        "frames.hex": "".join(wire.encode(d.measurement).hex() + "\n" for d in dets),
        "truth.csv": "\n".join(truth_rows(s)) + "\n",
    }


def render(s: scenarios.Scenario | scenarios.MultiScenario) -> dict[str, str]:
    """The three output files as text, keyed by file name."""
    if isinstance(s, scenarios.MultiScenario):
        return render_multi(s)
    samples = radar.observe(s.radar, s.trajectory, s.start_ns)
    frames = "".join(wire.encode(x.measurement).hex() + "\n" for x in samples)
    truth = [TRUTH_HEADER_V1]
    for x in samples:
        t = x.truth
        truth.append(f"{x.measurement.time_ns},{t.x_m!r},{t.y_m!r},{t.vx_mps!r},{t.vy_mps!r}")
    return {
        "sensors.cfg": sensors.format_table([s.radar.config]),
        "frames.hex": frames,
        "truth.csv": "\n".join(truth) + "\n",
    }


def write(s: scenarios.Scenario | scenarios.MultiScenario, out_dir: pathlib.Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, text in render(s).items():
        # newline="\n" keeps the bytes identical on every OS.
        (out_dir / name).write_text(text, encoding="ascii", newline="\n")


ALL = {**scenarios.SCENARIOS, **scenarios.MULTI_SCENARIOS}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted_sim.generate", description=__doc__.splitlines()[0])
    ap.add_argument("scenario", nargs="?", choices=sorted(ALL))
    ap.add_argument("out_dir", nargs="?", type=pathlib.Path)
    ap.add_argument("--list", action="store_true", help="list scenario names and exit")
    args = ap.parse_args(argv)
    if args.list:
        print("\n".join(sorted(ALL)))
        return 0
    if args.scenario is None or args.out_dir is None:
        ap.error("scenario and out_dir are required")
    write(ALL[args.scenario](), args.out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
