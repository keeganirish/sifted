"""Tables (Markdown) and plots for docs/benchmarks.md from the benchmark CSVs.

    python3 benchmarks/report.py docs/benchmarks            # reads *.csv there
    python3 benchmarks/report.py docs/benchmarks --no-plot

Inputs (any may be missing): engine.csv and engine_no_screen.csv from
`sifted_bench`, e2e.csv from e2e_latency.py. Prints the Markdown tables used
in docs/benchmarks.md; writes scan_latency.png next to the CSVs if matplotlib
is installed (it is optional: the tables are the report).
"""

from __future__ import annotations

import argparse
import csv
import pathlib
import sys


def rows(path: pathlib.Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def engine_table(eng: list[dict[str, str]], base: list[dict[str, str]]) -> str:
    before = {(r["targets"], r["clutter"]): r for r in base if r["bench"] == "scan"}
    out = ["| targets | clutter/scan | live tracks | scan size | median ms (spread) | p95 ms | p99 ms "
           "| max ms | µs/measurement | before (no screen) median ms |",
           "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for r in eng:
        if r["bench"] != "scan":
            continue
        b = before.get((r["targets"], r["clutter"]))
        out.append(f"| {r['targets']} | {r['clutter']} | {r['live_tracks']} | {r['scan_size']} | "
                   f"{float(r['scan_median_ms']):.3f} ({float(r['scan_median_min_ms']):.3f}-"
                   f"{float(r['scan_median_max_ms']):.3f}) | {float(r['scan_p95_ms']):.3f} | "
                   f"{float(r['scan_p99_ms']):.3f} | {float(r['scan_max_ms']):.3f} | "
                   f"{float(r['us_per_measurement']):.2f} | "
                   f"{float(b['scan_median_ms']):.3f} |" if b else
                   f"| {r['targets']} | {r['clutter']} | {r['live_tracks']} | {r['scan_size']} | "
                   f"{float(r['scan_median_ms']):.3f} ({float(r['scan_median_min_ms']):.3f}-"
                   f"{float(r['scan_median_max_ms']):.3f}) | {float(r['scan_p95_ms']):.3f} | "
                   f"{float(r['scan_p99_ms']):.3f} | {float(r['scan_max_ms']):.3f} | "
                   f"{float(r['us_per_measurement']):.2f} | - |")
    return "\n".join(out)


def capacity_table(eng: list[dict[str, str]]) -> str:
    out = ["| targets | clutter/scan | live tracks | p99 scan ms |", "|---:|---:|---:|---:|"]
    for r in eng:
        if r["bench"] == "capacity_probe":
            out.append(f"| {r['targets']} | {r['clutter']} | {r['live_tracks']} | "
                       f"{float(r['scan_p99_ms']):.2f} |")
    res = [r for r in eng if r["bench"] == "capacity_result"]
    if res:
        out.append(f"\n**Result:** {res[-1]['targets']} targets per engine thread "
                   f"(p99 ≤ {res[-1]['scan_median_ms']} ms).")
    return "\n".join(out)


def e2e_table(e2e: list[dict[str, str]]) -> str:
    out = ["| targets | reorder window s | scans | median ms | p95 ms | p99 ms | max ms |",
           "|---:|---:|---:|---:|---:|---:|---:|"]
    for r in e2e:
        out.append(f"| {r['targets']} | {r['reorder_window_s']} | {r['scans_measured']} | "
                   f"{r['median_ms']} | {r['p95_ms']} | {r['p99_ms']} | {r['max_ms']} |")
    return "\n".join(out)


def plot(directory: pathlib.Path, eng: list[dict[str, str]], base: list[dict[str, str]]) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("report: matplotlib not installed; no plot written", file=sys.stderr)
        return
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for data, label, style in ((eng, "gate screen (M3)", "-"), (base, "no screen (M2)", "--")):
        for ratio, colour in (("0", "#1f77b4"), ("0.1", "#ff7f0e"), ("1", "#2ca02c")):
            pts = []
            for r in data:
                if r["bench"] != "scan":
                    continue
                t, c = int(r["targets"]), int(r["clutter"])
                if (ratio == "0" and c == 0) or (ratio == "0.1" and c == t // 10 and c) or \
                        (ratio == "1" and c == t):
                    pts.append((int(r["live_tracks"]), float(r["scan_p99_ms"])))
            if pts:
                pts.sort()
                ax.plot([p[0] for p in pts], [p[1] for p in pts], style, marker="o", color=colour,
                        label=f"{label}, clutter {ratio}x targets")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("live tracks")
    ax.set_ylabel("p99 scan processing time [ms]")
    ax.axhline(100, color="grey", linewidth=0.8)
    ax.text(12, 110, "100 ms budget", color="grey", fontsize=8)
    ax.legend(fontsize=7)
    ax.grid(True, which="both", linewidth=0.3)
    fig.tight_layout()
    fig.savefig(directory / "scan_latency.png", dpi=110)
    print(f"report: wrote {directory / 'scan_latency.png'}", file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("directory", type=pathlib.Path)
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()
    eng = rows(a.directory / "engine.csv")
    base = rows(a.directory / "engine_no_screen.csv")
    e2e = rows(a.directory / "e2e.csv")
    print("### Per-scan engine latency\n\n" + engine_table(eng, base) + "\n")
    thr = [r for r in eng if r["bench"] == "throughput"]
    if thr:
        r = thr[-1]
        print(f"### Throughput\n\n{float(r['measurements_per_s']):.0f} measurements/s "
              f"({float(r['us_per_measurement']):.2f} µs each), {r['targets']} targets + "
              f"{r['clutter']} clutter/scan, {r['live_tracks']} live tracks.\n")
    print("### Tracks per core\n\n" + capacity_table(eng) + "\n")
    if e2e:
        print("### End to end\n\n" + e2e_table(e2e) + "\n")
    if not a.no_plot and eng:
        plot(a.directory, eng, base)
    return 0


if __name__ == "__main__":
    sys.exit(main())
