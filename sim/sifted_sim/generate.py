"""Writes a scenario's sensor table, encoded frames and ground truth to a directory.

    python3 -m sifted_sim.generate <scenario> <out_dir>
    python3 -m sifted_sim.generate --list

Outputs (consumed by tests/scenario_regression_test.cpp):

    sensors.cfg   sensor table, "sifted-sensors 1" text format (ADR 0003)
    frames.hex    one wire-v1 frame per line, hex-encoded, in send order
    truth.csv     time_ns,x_m,y_m,vx_mps,vy_mps; one row per frame, same order

Floats in truth.csv are written with repr() (shortest round-trip form).
"""

from __future__ import annotations

import argparse
import pathlib
import sys

from . import radar, scenarios, sensors, wire


def render(s: scenarios.Scenario) -> dict[str, str]:
    """The three output files as text, keyed by file name."""
    samples = radar.observe(s.radar, s.trajectory, s.start_ns)
    frames = "".join(wire.encode(x.measurement).hex() + "\n" for x in samples)
    truth = ["time_ns,x_m,y_m,vx_mps,vy_mps"]
    for x in samples:
        t = x.truth
        truth.append(f"{x.measurement.time_ns},{t.x_m!r},{t.y_m!r},{t.vx_mps!r},{t.vy_mps!r}")
    return {
        "sensors.cfg": sensors.format_table([s.radar.config]),
        "frames.hex": frames,
        "truth.csv": "\n".join(truth) + "\n",
    }


def write(s: scenarios.Scenario, out_dir: pathlib.Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, text in render(s).items():
        # newline="\n" keeps the bytes identical on every OS.
        (out_dir / name).write_text(text, encoding="ascii", newline="\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted_sim.generate", description=__doc__.splitlines()[0])
    ap.add_argument("scenario", nargs="?", choices=sorted(scenarios.SCENARIOS))
    ap.add_argument("out_dir", nargs="?", type=pathlib.Path)
    ap.add_argument("--list", action="store_true", help="list scenario names and exit")
    args = ap.parse_args(argv)
    if args.list:
        print("\n".join(sorted(scenarios.SCENARIOS)))
        return 0
    if args.scenario is None or args.out_dir is None:
        ap.error("scenario and out_dir are required")
    write(scenarios.SCENARIOS[args.scenario](), args.out_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
