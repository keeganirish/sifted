"""Writes Monte Carlo sets of the parametric manoeuvre scenario (ADR 0014).

    python3 -m sifted_sim.manoeuvres evaluation <out_dir>
    python3 -m sifted_sim.manoeuvres tuning <out_dir>

Each condition (turn load x speed) is written once per noise seed, as
`<out_dir>/<condition>_s<seed>/{sensors.cfg,frames.hex,truth.csv}` (the
generate.py layout), so `sifted_motion_eval --scenarios <out_dir>` can group
runs by condition. The sets are large and regenerable: write them into the
build directory, never into tests/fixtures.

Two disjoint sets, so parameters are never chosen on the cases reported:

    evaluation  right turns of 0.3, 1, 2, 3, 5 g at 100 and 250 m/s
                (and 7 g at 250 m/s), seeds 101-110. Reported in
                docs/motion-model-evaluation.md.
    tuning      left turns and S-turns (left then right) at 1.5 and 4 g,
                at 150 and 200 m/s, seeds 901-905. Used only to check the
                physically chosen IMM parameters before evaluation.
    reseeded    the committed multi-target fixture scenarios (crossing_pair,
                close_pass, enter_leave, formation_split, demo) with ten new
                noise seeds each: is a fixture difference the estimator, or
                the fixture's one noise draw?
"""

from __future__ import annotations

import argparse
import dataclasses
import pathlib
import sys

from . import generate, scenarios
from .scenarios import ManoeuvreLeg

EVALUATION_LOADS = (0.3, 1.0, 2.0, 3.0, 5.0)
EVALUATION_SPEEDS = (100.0, 250.0)
EVALUATION_EXTRA = ((7.0, 250.0),)  # 7 g at 100 m/s would turn 180 deg in 4.6 s
EVALUATION_SEEDS = tuple(range(101, 111))

TUNING_LOADS = (1.5, 4.0)
TUNING_SPEEDS = (150.0, 200.0)
TUNING_SEEDS = tuple(range(901, 906))


def evaluation_set() -> list[tuple[str, scenarios.MultiScenario]]:
    out = []
    conds = [(g, v) for v in EVALUATION_SPEEDS for g in EVALUATION_LOADS] + list(EVALUATION_EXTRA)
    for g, v in conds:
        for seed in EVALUATION_SEEDS:
            s = scenarios.manoeuvre(g, v, seed)
            out.append((s.name, s))
    return out


def tuning_set() -> list[tuple[str, scenarios.MultiScenario]]:
    out = []
    for v in TUNING_SPEEDS:
        for g in TUNING_LOADS:
            for label, legs in (("left", (ManoeuvreLeg(g, right=False),)),
                                ("s", (ManoeuvreLeg(g, right=False), ManoeuvreLeg(g)))):
                for seed in TUNING_SEEDS:
                    s = scenarios.manoeuvre(g, v, seed, legs=legs)
                    out.append((f"tuning_{label}_{g:g}g_{v:g}mps_s{seed}", s))
    return out


RESEEDED_SEEDS = tuple(range(1, 11))


def reseeded_set() -> list[tuple[str, scenarios.MultiScenario]]:
    """The committed multi-target fixture scenarios with new noise seeds.

    Same geometry, sensors and trajectories; every sensor's seed is offset, so
    noise, detections and clutter differ from the committed fixture. Shows
    whether a difference on a fixture is the estimator or that one noise draw.
    """
    out = []
    for name, make in sorted(scenarios.MULTI_SCENARIOS.items()):
        base = make()
        for k in RESEEDED_SEEDS:
            s = dataclasses.replace(
                base,
                radars=[dataclasses.replace(r, seed=r.seed + 100_000 * k) for r in base.radars],
                cameras=[dataclasses.replace(c, seed=c.seed + 100_000 * k) for c in base.cameras])
            out.append((f"{name}_s{k}", s))
    return out


SETS = {"evaluation": evaluation_set, "tuning": tuning_set, "reseeded": reseeded_set}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted_sim.manoeuvres", description=__doc__.splitlines()[0])
    ap.add_argument("set", choices=sorted(SETS))
    ap.add_argument("out_dir", type=pathlib.Path)
    args = ap.parse_args(argv)
    for name, s in SETS[args.set]():
        generate.write(s, args.out_dir / name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
