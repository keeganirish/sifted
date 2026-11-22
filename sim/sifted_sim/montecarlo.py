"""Monte Carlo sets: independent noise realisations of the multi-target scenarios (ADR 0015).

    python3 -m sifted_sim.montecarlo generate <out_dir> [--smoke]
    python3 -m sifted_sim.montecarlo report [--smoke] [--work DIR] [--binary PATH] [--out FILE]

`generate` writes every run as `<out_dir>/<set>/<condition>/<scenario>/rNNN/`
(the generate.py layout: sensors.cfg, frames.hex, truth.csv) plus
`<out_dir>/manifest.csv`, which `sifted_montecarlo` reads. The sets are large
and regenerable: write them into the build directory, never into the
repository. `report` generates the default sets into build/montecarlo (skipped
when that manifest is already current) and runs
build/release/eval/sifted_montecarlo to write docs/evaluation-monte-carlo.md.

Sets (each run is one independent realisation of every sensor's noise,
detections and clutter; geometry, trajectories and sensor parameters are the
scenario's own unless the condition says otherwise):

    noise    every multi-target scenario (scenarios.MULTI_SCENARIOS) as defined,
             NOISE_RUNS runs each.
    sweep    every multi-target scenario with each radar's clutter_per_scan and
             p_detect replaced by each of SWEEP_CLUTTER x SWEEP_PD (cameras
             unchanged: they have no clutter model), SWEEP_RUNS runs each, so
             SWEEP_RUNS x 5 runs per condition.

Seed scheme. mix(x) is one SplitMix64 output for state x (Steele, Lea and
Flood, 2014; impair.SplitMix64), fnv(s) is 64-bit FNV-1a of the UTF-8 name:

    scenario_key = mix(mix(base_seed) ^ fnv(scenario))
    run_seed     = mix(scenario_key ^ mix(run))
    sensor_seed  = mix(run_seed ^ (sensor_id * 0xD1B54A32D192ED03 mod 2^64))
    impair_seed  = mix(run_seed ^ fnv("impairment"))

Every input passes through a full-avalanche mixer, so adjacent run indices,
scenarios or sensor ids give unrelated 64-bit seeds rather than the
consecutive small integers whose Mersenne Twister streams are the classic
correlated-seed trap. The sensors seed random.Random with sensor_seed (noise)
and sensor_seed ^ 0x5EEDF00D (detections and clutter); tests check that no
two of these collide across the default sets and that first draws of
neighbouring runs are uncorrelated. The seed depends on scenario and run
only, never on the set or condition: run r of a scenario has the same seeds
in every condition (common random numbers), so differences between conditions
and between trackers can be analysed in pairs.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import os
import pathlib
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor

from . import generate, scenarios
from .impair import MASK64, SplitMix64

DEFAULT_BASE_SEED = 20261119
NOISE_RUNS = 100
SWEEP_CLUTTER = (0.5, 5.0, 20.0, 50.0, 100.0)
SWEEP_PD = (0.9, 0.7)
SWEEP_RUNS = 20  # per scenario: 100 per condition over the five scenarios

# --smoke: a few runs, for tests and a quick look. Same seeds as the full sets.
SMOKE_SCENARIOS = ("close_pass", "formation_split")
SMOKE_NOISE_RUNS = 3
SMOKE_SWEEP = ((0.5, 0.9), (20.0, 0.7))
SMOKE_SWEEP_RUNS = 1

MANIFEST_HEADER = "sifted-montecarlo 1"
MANIFEST_COLUMNS = "set,condition,scenario,run,seed,impairment_seed,p_detect,clutter_per_scan,dir"
EVENTS_XOR = 0x5EED_F00D  # radar.py / camera.py: the detection-and-clutter stream


def mix(x: int) -> int:
    return SplitMix64(x & MASK64).next()


def fnv1a64(name: str) -> int:
    h = 0xCBF29CE484222325
    for b in name.encode("utf-8"):
        h = ((h ^ b) * 0x100000001B3) & MASK64
    return h


def run_seed(base_seed: int, scenario: str, run: int) -> int:
    key = mix(mix(base_seed) ^ fnv1a64(scenario))
    return mix(key ^ mix(run))


def sensor_seed(seed: int, sensor_id: int) -> int:
    return mix(seed ^ ((sensor_id * 0xD1B54A32D192ED03) & MASK64))


def impairment_seed(seed: int) -> int:
    return mix(seed ^ fnv1a64("impairment"))


@dataclasses.dataclass(frozen=True)
class Run:
    set: str
    condition: str
    scenario: str
    run: int
    seed: int
    p_detect: float | None = None  # None: the scenario's own radars
    clutter_per_scan: float | None = None

    @property
    def dir(self) -> str:
        return f"{self.set}/{self.condition}/{self.scenario}/r{self.run:03d}"

    def build(self) -> scenarios.MultiScenario:
        """The scenario with this run's seeds (and sweep parameters)."""
        base = scenarios.MULTI_SCENARIOS[self.scenario]()
        sweep = {} if self.p_detect is None else {"p_detect": self.p_detect,
                                                    "clutter_per_scan": self.clutter_per_scan}
        return dataclasses.replace(
            base,
            radars=[dataclasses.replace(r, seed=sensor_seed(self.seed, r.sensor_id), **sweep)
                    for r in base.radars],
            cameras=[dataclasses.replace(c, seed=sensor_seed(self.seed, c.sensor_id))
                     for c in base.cameras])


def sweep_condition(clutter: float, pd: float) -> str:
    return f"clutter{clutter:g}_pd{pd:g}"


def plan(base_seed: int = DEFAULT_BASE_SEED, smoke: bool = False) -> list[Run]:
    """Every run of the sets, in manifest order."""
    names = SMOKE_SCENARIOS if smoke else tuple(sorted(scenarios.MULTI_SCENARIOS))
    noise_runs = SMOKE_NOISE_RUNS if smoke else NOISE_RUNS
    sweep = SMOKE_SWEEP if smoke else tuple((c, p) for p in SWEEP_PD for c in SWEEP_CLUTTER)
    sweep_runs = SMOKE_SWEEP_RUNS if smoke else SWEEP_RUNS
    out = []
    for name in names:
        for r in range(noise_runs):
            out.append(Run("noise", "default", name, r, run_seed(base_seed, name, r)))
    for clutter, pd in sweep:
        for name in names:
            for r in range(sweep_runs):
                out.append(Run("sweep", sweep_condition(clutter, pd), name, r,
                               run_seed(base_seed, name, r), pd, clutter))
    return out


def _generator_digest() -> str:
    """SHA-256 of this package's sources: a manifest from other code is stale."""
    h = hashlib.sha256()
    for p in sorted(pathlib.Path(__file__).parent.glob("*.py")):
        h.update(p.name.encode() + b"\0" + p.read_bytes() + b"\0")
    return h.hexdigest()


def manifest_text(runs: list[Run], base_seed: int) -> str:
    def opt(v: float | None) -> str:
        return "" if v is None else repr(v)

    lines = [MANIFEST_HEADER, f"base_seed,{base_seed}", f"generator,{_generator_digest()}",
             MANIFEST_COLUMNS]
    for r in runs:
        lines.append(f"{r.set},{r.condition},{r.scenario},{r.run},{r.seed},"
                     f"{impairment_seed(r.seed)},{opt(r.p_detect)},{opt(r.clutter_per_scan)},{r.dir}")
    return "\n".join(lines) + "\n"


def _write_one(args: tuple[Run, str]) -> None:
    run, out_dir = args
    generate.write(run.build(), pathlib.Path(out_dir) / run.dir)


def write_sets(out_dir: pathlib.Path, base_seed: int = DEFAULT_BASE_SEED, smoke: bool = False,
               jobs: int | None = None, force: bool = False) -> pathlib.Path:
    """Writes every run and the manifest; returns the manifest path.

    Skipped when the manifest already there is identical (same runs, seeds and
    generator sources) and `force` is false. Output does not depend on `jobs`.
    """
    runs = plan(base_seed, smoke)
    text = manifest_text(runs, base_seed)
    manifest = out_dir / "manifest.csv"
    if not force and manifest.is_file() and manifest.read_text(encoding="ascii") == text and all(
            (out_dir / r.dir / "truth.csv").is_file() for r in runs):
        return manifest
    out_dir.mkdir(parents=True, exist_ok=True)
    if manifest.exists():
        manifest.unlink()  # never leave a manifest describing a half-written set
    work = [(r, str(out_dir)) for r in runs]
    if jobs == 1:
        for w in work:
            _write_one(w)
    else:
        with ProcessPoolExecutor(max_workers=jobs or os.cpu_count()) as pool:
            list(pool.map(_write_one, work, chunksize=8))
    manifest.write_text(text, encoding="ascii", newline="\n")
    return manifest


REPO = pathlib.Path(__file__).resolve().parents[2]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted_sim.montecarlo", description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    g = sub.add_parser("generate", help="write the Monte Carlo sets and manifest")
    g.add_argument("out_dir", type=pathlib.Path)
    rep = sub.add_parser("report", help="generate (if needed) and write the Monte Carlo report")
    rep.add_argument("--work", type=pathlib.Path, default=None,
                     help="where the sets go (default build/montecarlo, or build/montecarlo-smoke)")
    rep.add_argument("--binary", type=pathlib.Path,
                     default=REPO / "build/release/eval/sifted_montecarlo")
    rep.add_argument("--out", type=pathlib.Path, default=None,
                     help="report path (default docs/evaluation-monte-carlo.md; with --smoke, "
                          "<work>/evaluation-monte-carlo.md)")
    for p in (g, rep):
        p.add_argument("--smoke", action="store_true", help="a few runs only")
        p.add_argument("--base-seed", type=int, default=DEFAULT_BASE_SEED)
        p.add_argument("--jobs", type=int, default=None, help="generator processes")
        p.add_argument("--force", action="store_true", help="regenerate even if current")
    args = ap.parse_args(argv)
    if args.cmd == "generate":
        write_sets(args.out_dir, args.base_seed, args.smoke, args.jobs, args.force)
        return 0
    work = args.work or REPO / ("build/montecarlo-smoke" if args.smoke else "build/montecarlo")
    manifest = write_sets(work, args.base_seed, args.smoke, args.jobs, args.force)
    out = args.out or (work / "evaluation-monte-carlo.md" if args.smoke
                       else REPO / "docs/evaluation-monte-carlo.md")
    if not args.binary.is_file():
        sys.exit(f"{args.binary}: not built (cmake --preset release && cmake --build --preset release)")
    cmd = [str(args.binary), "--manifest", str(manifest), "--out", str(out)]
    return subprocess.run(cmd, check=False).returncode


if __name__ == "__main__":
    sys.exit(main())
