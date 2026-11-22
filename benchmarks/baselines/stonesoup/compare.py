"""Sifted against a Stone Soup baseline, on the same Monte Carlo runs and scorer.

    build/stonesoup-venv/bin/python benchmarks/baselines/stonesoup/compare.py
        [--smoke] [--full] [--jobs N] [--binary PATH]

1. writes the Monte Carlo sets (sim/sifted_sim/montecarlo.py; reused when
   current) and a subset manifest: the first RUNS_PER_SCENARIO[...] runs of each
   scenario in each condition (the same runs, seeds and files Sifted gets);
2. runs tracker.py on every run in parallel processes (one thread each),
   writing a track report per run plus tracker.txt (versions and settings);
3. runs sifted_montecarlo with Sifted's in-process configurations and the
   Stone Soup reports, which writes docs/evaluation-baseline.md;
4. writes docs/benchmarks/baseline-timing.md from both trackers' per-run
   processing times (machine dependent, so kept off the deterministic page).

--smoke uses the smoke sets and writes everything under build/baseline-smoke.
--full uses 20 runs per scenario (100 per condition) everywhere, which takes
hours at heavy clutter (README.md, "Run counts").
"""

from __future__ import annotations

import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")  # one thread per run; the runs are parallel

import argparse  # noqa: E402
import csv  # noqa: E402
import pathlib  # noqa: E402
import platform  # noqa: E402
import statistics  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
from concurrent.futures import ProcessPoolExecutor  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "sim"))
sys.path.insert(0, str(HERE))
from sifted_sim import montecarlo  # noqa: E402

import tracker  # noqa: E402

# Runs per scenario (five scenarios) in each condition, by condition prefix.
# Stone Soup's cost per scan grows with tentative tracks x detections, so
# heavy clutter is subsampled (README.md, "Run counts").
RUNS_PER_SCENARIO = {
    "default": 20,
    "clutter0.5_": 20,
    "clutter5_": 20,
    "clutter20_": 20,
    "clutter50_": 4,
    "clutter100_": 2,
}


def runs_for(condition: str, full: bool) -> int:
    if full:
        return 20
    for prefix, n in RUNS_PER_SCENARIO.items():
        if condition.startswith(prefix):
            return n
    raise SystemExit(f"no run count for condition {condition!r}")


def subset_manifest(manifest: pathlib.Path, full: bool, smoke: bool) -> pathlib.Path:
    lines = manifest.read_text(encoding="ascii").splitlines()
    head, rows = lines[:4], lines[4:]
    keep = []
    for row in rows:
        f = row.split(",")
        condition, run = f[1], int(f[3])
        if smoke or run < runs_for(condition, full):
            keep.append(row)
    out = manifest.with_name("baseline-manifest.csv")
    out.write_text("\n".join(head + keep) + "\n", encoding="ascii", newline="\n")
    return out


def _track_one(args: tuple[str, str, str]) -> tuple[str, float, int]:
    root, rel, out_root = args
    text, stats = tracker.track(pathlib.Path(root) / rel)
    out = pathlib.Path(out_root) / (rel + ".tracks")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="ascii", newline="\n")
    return rel, stats.seconds, stats.track_limit


def versions() -> dict[str, str]:
    import numpy
    import scipy
    import stonesoup
    return {"stonesoup": stonesoup.__version__, "numpy": numpy.__version__,
            "scipy": scipy.__version__, "python": platform.python_version()}


def describe(v: dict[str, str]) -> str:
    c = tracker.Config()
    return (f"Stone Soup {v['stonesoup']} (Python {v['python']}, numpy {v['numpy']}, scipy "
            f"{v['scipy']}): EKF (ExtendedKalmanPredictor/Updater, Joseph form), constant "
            f"velocity q = {c.accel_psd:g} m^2/s^3, range-bearing and bearing-only updates, "
            f"squared-Mahalanobis chi-square gate {c.gate_probability:g}, global nearest "
            f"neighbour (GNNWith2DAssignment), MultiMeasurementInitiator ({c.confirm_hits} hits, "
            f"{c.tentative_timeout_s:g} s), deletion {c.drop_after_s:g} s after the last "
            f"range-bearing update or at position sigma > {c.max_position_sigma_m:g} m; "
            "not tuned (benchmarks/baselines/stonesoup)")


def cpu_name() -> str:
    try:
        for line in pathlib.Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or "unknown"


def timing_page(rows: list[dict], v: dict[str, str], jobs: int, cmd: str) -> str:
    by: dict[tuple[str, str], list[float]] = {}
    for r in rows:
        by.setdefault((r["condition"], r["tracker"]), []).append(float(r["seconds"]))
    conds = sorted({c for c, _ in by}, key=lambda c: (c != "default", c))
    trackers = sorted({t for _, t in by}, key=lambda t: (t == "stonesoup", t))
    out = ["# Baseline processing time", "",
           "<!-- Generated by benchmarks/baselines/stonesoup/compare.py. Machine dependent: "
           "not byte-reproducible, unlike docs/evaluation-baseline.md. -->", "",
           "Wall-clock processing time per run, on the runs of "
           "[evaluation-baseline.md](../evaluation-baseline.md). What is timed is the same for "
           "both: every measurement consumed scan by scan and the tracks extrapolated to every "
           "truth instant. Reading and decoding the files and scoring are not timed.", "",
           "| | |", "|---|---|",
           f"| CPU | {cpu_name()}, {os.cpu_count()} logical CPUs |",
           f"| OS | {platform.system()} {platform.release()} |",
           "| Sifted | C++20, `release` preset (`-O3`, `-ffp-contract=off`), single thread, runs "
           "one after another (`sifted_montecarlo --timing`) |",
           f"| Stone Soup | {v['stonesoup']}, CPython {v['python']}, numpy {v['numpy']}, one "
           f"thread per run (BLAS threads 1), {jobs} runs in parallel processes |", "",
           "Limits of this comparison: the languages differ (compiled C++ against interpreted "
           "Python on numpy), Stone Soup is a general framework built for clarity and "
           "flexibility (objects per state, hypothesis and detection), and parallel processes "
           "share caches and memory bandwidth, which slows each a little. The ratio shows the "
           "order of magnitude, not a property of either algorithm.", "",
           "| condition | tracker | runs | median s | 90th percentile s | max s |",
           "|---|---|---:|---:|---:|---:|"]
    for c in conds:
        for t in trackers:
            xs = by.get((c, t))
            if not xs:
                continue
            xs.sort()
            p90 = xs[min(len(xs) - 1, int(0.9 * (len(xs) - 1) + 0.5))]
            out.append(f"| {c} | {t} | {len(xs)} | {statistics.median(xs):.4f} | {p90:.4f} | "
                       f"{xs[-1]:.4f} |")
    out += ["", "Reproduce:", "", "```bash", cmd, "```", ""]
    return "\n".join(out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--smoke", action="store_true")
    ap.add_argument("--full", action="store_true", help="20 runs per scenario everywhere (hours)")
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    ap.add_argument("--binary", type=pathlib.Path,
                    default=REPO / "build/release/eval/sifted_montecarlo")
    args = ap.parse_args(argv)
    if not args.binary.is_file():
        sys.exit(f"{args.binary}: not built (cmake --preset release && cmake --build --preset release)")

    sets = REPO / ("build/montecarlo-smoke" if args.smoke else "build/montecarlo")
    work = REPO / ("build/baseline-smoke" if args.smoke else "build/baseline")
    manifest = montecarlo.write_sets(sets, smoke=args.smoke)
    subset = subset_manifest(manifest, args.full, args.smoke)
    rels = [line.split(",")[-1] for line in subset.read_text(encoding="ascii").splitlines()[4:]]
    tracks = work / "stonesoup"
    tracks.mkdir(parents=True, exist_ok=True)
    v = versions()
    (tracks / "tracker.txt").write_text(describe(v) + "\n", encoding="ascii", newline="\n")

    jobs = max(1, args.jobs)
    timing_rows = []
    cond_of = {line.split(",")[-1]: line.split(",")[1]
               for line in subset.read_text(encoding="ascii").splitlines()[4:]}
    # Longest first, so the slowest runs do not start last.
    def clutter(rel: str) -> float:
        cond = rel.split("/")[1]
        return float(cond[len("clutter"):].split("_")[0]) if cond.startswith("clutter") else 0.0

    order = sorted(rels, key=lambda r: (-clutter(r), r))
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        for rel, seconds, limited in pool.map(_track_one,
                                              [(str(sets), r, str(tracks)) for r in order]):
            timing_rows.append({"tracker": "stonesoup", "condition": cond_of[rel],
                                "seconds": seconds, "limited": limited})
            print(f"{rel}: {seconds:.2f} s", flush=True)

    page = REPO / "docs/evaluation-baseline.md" if not args.smoke else work / "evaluation-baseline.md"
    timing_csv = work / "sifted-timing.csv"
    cmd = [str(args.binary), "--manifest", str(subset),
           "--trackers", "sifted,sifted-cv,sifted-cv-fixed-q",
           "--tracks", f"stonesoup={tracks}", "--reference", "stonesoup", "--network", "none",
           "--page", "baseline", "--timing", str(timing_csv), "--out", str(page)]
    rc = subprocess.run(cmd, check=False).returncode
    if rc != 0:
        return rc
    with timing_csv.open(encoding="ascii") as f:
        for r in csv.DictReader(f):
            timing_rows.append({"tracker": r["tracker"], "condition": r["condition"],
                                "seconds": r["seconds"]})
    timing = (REPO / "docs/benchmarks/baseline-timing.md" if not args.smoke
              else work / "baseline-timing.md")
    timing.write_text(timing_page(
        timing_rows, v, jobs,
        "build/stonesoup-venv/bin/python benchmarks/baselines/stonesoup/compare.py"),
        encoding="utf-8", newline="\n")
    print(f"wrote {page.relative_to(REPO)} and {timing.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
