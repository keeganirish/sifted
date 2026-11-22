"""Real-data evaluation: Sifted against recorded OpenSky ADS-B (docs/real-data-evaluation.md).

    python3 -m sifted_sim.realeval fetch      # download the manifest's archives (the only network step)
    python3 -m sifted_sim.realeval run        # cached data -> docs/real-data-evaluation.md
    python3 -m sifted_sim.realeval describe FILE...   # descriptive manifest fields of archives

Run from sim/ (or with sim/ on PYTHONPATH). `run` needs the release build
(`cmake --preset release && cmake --build --preset release`) for sifted_eval
and sifted_realeval; --bin-dir points elsewhere. Defaults: manifest
eval/realdata/opensky-incident.json, cache recordings/opensky-incident/raw
(git-ignored), work directory build/realdata (git-ignored).

Two complementary evaluations, both on the same cleaned flights:

  A  real measurements. Real reports through the real wire codec, decoder and
     engine as GeoPosition measurements. There is no independent truth, so
     scoring is predictive: a deterministic schedule withholds runs of each
     aircraft's reports, and the tracker's estimate at each withheld report's
     time is compared with it (sifted_realeval, eval/sifted/eval/realdata.hpp),
     against two baselines: holding the last report, and dead reckoning it by
     its reported velocity. The aircraft address never reaches the tracker:
     frames carry time, position and sensor only; addresses become small
     integer labels in scoring-only files.
  B  real trajectories, simulated sensors. Cleaned flight paths, interpolated
     (sifted_sim.recorded), are the truth for the existing seeded radar and
     camera simulators, written as standard scenario directories and scored
     by the existing sifted_eval. A constant-velocity control with the same
     sensors, noise and seeds isolates the error due to real motion.

Every selection rule and parameter is a constant below or in realclean.py,
declared before any result was seen, and printed in the report. Flights are
split by start time into a tuning portion (every TUNING_EVERY-th flight) and
a held-out portion; the only data-driven choice (the interpolation method)
is made on the tuning flights, and headline results are the held-out ones.
The report is a deterministic function of the cached data and the binaries:
no wall-clock time, sorted iteration, fixed number formatting.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import pathlib
import shutil
import statistics
import subprocess
import sys
import urllib.request
from collections import Counter
from dataclasses import dataclass, field

from . import adsb, generate, geo, realclean, recorded, sensors, wire
from .camera import CameraParams
from .radar import RadarParams
from .scenarios import RATE_HZ, SIGMA_BEARING_RAD, SIGMA_RANGE_M, MultiScenario
from .statevectors import Reader, Report
from .trajectory import Straight, Target, TargetState, Trajectory

REPO = pathlib.Path(__file__).resolve().parents[2]
DEFAULT_MANIFEST = REPO / "eval" / "realdata" / "opensky-incident.json"
DEFAULT_CACHE = REPO / "recordings" / "opensky-incident" / "raw"
DEFAULT_WORK = REPO / "build" / "realdata"
DEFAULT_BIN = REPO / "build" / "release" / "eval"
DEFAULT_OUT = REPO / "docs" / "real-data-evaluation.md"
MAX_DOWNLOAD_BYTES = 256 * 1024 * 1024

# ---- declared parameters (recorded in the report) ---------------------------------

TUNING_EVERY = 3            # flights 0, 3, 6, ... by start time are the tuning portion
ADSB_SENSOR_ID = 100        # as app/launcher.py
SIGMA_M = 50.0              # the adapter's assumed horizontal 1-sigma (adsb.Adapter)
# The launcher's lifecycle settings for sparse ADS-B input (app/launcher.py
# ADSB_TRACKER_ARGS; a test keeps the two equal).
ADSB_TRACKER_ARGS = ["--initial-velocity-sigma", "250", "--tentative-timeout", "90",
                     "--coast-after", "60", "--drop-after", "180", "--max-position-sigma", "10000"]
# Live REST path emulation: the signed-in poll floor for a 1-credit box.
POLL_INTERVAL_S = adsb.min_interval_s(1, True)
# Withholding schedule, per aircraft and engine run: feed A_BURN_IN_S, then
# repeat (withhold one gap of A_GAPS_S in turn, feed A_FEED_S).
A_BURN_IN_S = 120.0
A_GAPS_S = (2.0, 5.0, 10.0, 20.0, 40.0, 80.0)
A_FEED_S = 60.0
HORIZON_EDGES_S = (0, 2, 5, 10, 20, 40, 80, 160)
# Evaluation B geometry: the simulator's sensors (sifted_sim.scenarios), with
# coverage scaled to aircraft speeds, placed beside each window's midpoint.
B_WINDOW_S = 300.0
B_KNOT_MARGIN_S = 30.0
B_MAX_KNOT_GAP_S = 10.0
B_RADAR_RANGE_M = 20_000.0
B_SENSOR_OFFSET_M = 5_000.0  # radar to the left of the track, camera to the right
B_CAMERA_RANGE_M = 30_000.0
B_SEEDS = (1, 2, 3)
B_TURN_DEG = 15.0           # total heading change above which a window is "manoeuvring"
# kMatchDistanceM's rule (several times the raw radar error at the coverage
# edge) applied to this radar: sqrt(10^2 + (20 km x 0.2 deg)^2) = 71 m, x ~4.
B_MATCH_DISTANCE_M = 300.0

PARAMETERS = {
    "TUNING_EVERY": TUNING_EVERY, "SIGMA_M": SIGMA_M, "POLL_INTERVAL_S": POLL_INTERVAL_S,
    "A_BURN_IN_S": A_BURN_IN_S, "A_GAPS_S": A_GAPS_S, "A_FEED_S": A_FEED_S,
    "HORIZON_EDGES_S": HORIZON_EDGES_S, "B_WINDOW_S": B_WINDOW_S,
    "B_KNOT_MARGIN_S": B_KNOT_MARGIN_S, "B_MAX_KNOT_GAP_S": B_MAX_KNOT_GAP_S,
    "B_RADAR_RANGE_M": B_RADAR_RANGE_M, "B_SENSOR_OFFSET_M": B_SENSOR_OFFSET_M,
    "B_CAMERA_RANGE_M": B_CAMERA_RANGE_M, "B_SEEDS": B_SEEDS, "B_TURN_DEG": B_TURN_DEG,
    "B_MATCH_DISTANCE_M": B_MATCH_DISTANCE_M,
}


class EvalError(RuntimeError):
    """A clear, user-facing failure (missing data, empty after filtering, tool error)."""


# ---- datasets ----------------------------------------------------------------------

def sha256_of(path: pathlib.Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def load_manifest(path: pathlib.Path) -> dict:
    try:
        m = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as e:
        raise EvalError(f"cannot read manifest {path}: {e}") from e
    if not isinstance(m, dict) or not isinstance(m.get("datasets"), list) or not m["datasets"]:
        raise EvalError(f"{path}: manifest needs a non-empty 'datasets' list")
    for d in m["datasets"]:
        for k in ("label", "file", "sha256", "bytes"):
            if k not in d:
                raise EvalError(f"{path}: dataset entry without {k!r}")
    return m


def fetch(manifest: dict, cache: pathlib.Path) -> None:
    """Downloads every dataset not already cached with the right SHA-256."""
    cache.mkdir(parents=True, exist_ok=True)
    for d in manifest["datasets"]:
        dest = cache / d["file"]
        if dest.exists() and sha256_of(dest) == d["sha256"]:
            print(f"realeval: {d['file']}: cached, checksum ok", file=sys.stderr)
            continue
        if d["bytes"] > MAX_DOWNLOAD_BYTES:
            raise EvalError(f"{d['file']}: larger than {MAX_DOWNLOAD_BYTES} bytes")
        tmp = dest.with_suffix(dest.suffix + ".part")
        with urllib.request.urlopen(d["url"], timeout=60) as r, open(tmp, "wb") as f:
            got = 0
            while block := r.read(1 << 20):
                got += len(block)
                if got > d["bytes"]:
                    raise EvalError(f"{d['file']}: more bytes than the manifest says")
                f.write(block)
        if sha256_of(tmp) != d["sha256"]:
            tmp.unlink()
            raise EvalError(f"{d['file']}: SHA-256 mismatch (truncated or changed upstream)")
        tmp.replace(dest)
        print(f"realeval: {d['file']}: downloaded, checksum ok", file=sys.stderr)


@dataclass
class Dataset:
    label: str
    entry: dict
    reports: list[Report]
    read_drops: Counter
    rows: int
    file_order_late: int  # valid reports older than one before them in the file


def _iso(ms: int) -> str:
    return datetime.datetime.fromtimestamp(ms // 1000, datetime.timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%SZ")


def read_dataset(path: pathlib.Path) -> tuple[list[Report], Reader, int]:
    rd = Reader()
    reports, newest, late = [], None, 0
    for _, r in rd.read_path(str(path)):
        if newest is not None and r.t_ms < newest:
            late += 1
        newest = r.t_ms if newest is None else max(newest, r.t_ms)
        reports.append(r)
    return reports, rd, late


def describe(reports: list[Report], rows: int) -> dict:
    """The descriptive manifest fields, computed from the data."""
    if not reports:
        return {"state_vectors": rows, "valid_reports": 0, "aircraft": 0}
    return {
        "state_vectors": rows,
        "valid_reports": len(reports),
        "aircraft": len({r.icao24 for r in reports}),
        "time_range_utc": [_iso(min(r.t_ms for r in reports)), _iso(max(r.t_ms for r in reports))],
        "area_deg": {"lat_min": round(min(r.lat for r in reports), 3),
                     "lat_max": round(max(r.lat for r in reports), 3),
                     "lon_min": round(min(r.lon for r in reports), 3),
                     "lon_max": round(max(r.lon for r in reports), 3)},
    }


DESCRIBED = ("state_vectors", "valid_reports", "aircraft", "time_range_utc", "area_deg")


def load_datasets(manifest: dict, cache: pathlib.Path) -> list[Dataset]:
    out = []
    for d in sorted(manifest["datasets"], key=lambda d: d["label"]):
        path = cache / d["file"]
        if not path.exists():
            raise EvalError(f"{path}: not cached; run `python3 -m sifted_sim.realeval fetch` "
                            "(or pass --cache)")
        if path.stat().st_size != d["bytes"] or sha256_of(path) != d["sha256"]:
            raise EvalError(f"{path}: size or SHA-256 differs from the manifest "
                            "(truncated download, or a different file)")
        reports, rd, late = read_dataset(path)
        got = describe(reports, rd.rows)
        stale = [k for k in DESCRIBED if k in d and d[k] != got.get(k)]
        if stale:
            raise EvalError(f"{d['file']}: manifest fields {stale} do not match the data; "
                            "regenerate them with `realeval describe`")
        out.append(Dataset(d["label"], d, reports, rd.dropped, rd.rows, late))
    return out


# ---- flights, labels and splits ----------------------------------------------------

@dataclass
class FlightInfo:
    name: str          # F01, F02, ... by start time
    split: str         # "tuning" | "heldout"
    aircraft: int      # label 1..N (never the address)
    flight: realclean.Flight


def label_flights(datasets: list[Dataset]) -> tuple[list[FlightInfo], dict, Counter]:
    """Clean every dataset; returns flights, removal counts per dataset, totals."""
    flights: list[realclean.Flight] = []
    removed: dict[str, Counter] = {}
    for ds in datasets:
        fl, rem = realclean.clean(ds.label, ds.reports)
        flights += fl
        removed[ds.label] = rem
    if not flights:
        raise EvalError("no flight left after cleaning: nothing to evaluate "
                        f"(removed: {dict(sum(removed.values(), Counter()))})")
    keys = sorted({(f.dataset, f.icao24) for f in flights})
    label = {k: i + 1 for i, k in enumerate(keys)}
    flights.sort(key=lambda f: (f.start_ms, f.dataset, f.icao24, f.index))
    infos = [FlightInfo(f"F{i + 1:02d}", "tuning" if i % TUNING_EVERY == 0 else "heldout",
                        label[(f.dataset, f.icao24)], f) for i, f in enumerate(flights)]
    return infos, removed, sum(removed.values(), Counter())


SPLITS = ("tuning", "heldout")


# ---- interpolation (leave-one-out) -------------------------------------------------

def knots_of(reports: list[Report] | tuple[Report, ...], enu: geo.Enu, t0_ms: int) -> list[recorded.Knot]:
    out = []
    for r in reports:
        x, y = enu.project(r.lat, r.lon)
        if r.speed_mps is not None and r.track_deg is not None:
            vx, vy = enu.velocity(r.lat, r.lon, r.speed_mps, r.track_deg)
        else:
            vx = vy = None
        out.append(recorded.Knot((r.t_ms - t0_ms) / 1000.0, x, y, vx, vy))
    return out


def flight_knots(f: realclean.Flight) -> list[recorded.Knot]:
    mid = f.reports[len(f.reports) // 2]
    return knots_of(f.reports, geo.Enu(mid.lat, mid.lon), f.start_ms)


LOO_EDGES_S = (0.0, 2.5, 5.0, 10.0, 20.0, 120.0)


def loo_stats(flights: list[FlightInfo], method: str) -> dict:
    samples = []
    for fi in flights:
        samples += recorded.leave_one_out(flight_knots(fi.flight), method)
    out = {"all": quant([e for _, e in samples])}
    for lo, hi in zip(LOO_EDGES_S, LOO_EDGES_S[1:]):
        out[(lo, hi)] = quant([e for s, e in samples if lo < s <= hi])
    return out


def quant(v: list[float]) -> dict:
    """Nearest-rank quantiles, as the C++ tool computes them (realdata.hpp)."""
    if not v:
        return {"n": 0}
    v = sorted(v)

    def rank(p: float) -> float:
        return v[min(max(math.ceil(p * len(v)), 1), len(v)) - 1]
    return {"n": len(v), "median": rank(0.5), "p95": rank(0.95), "p99": rank(0.99),
            "mean": math.fsum(v) / len(v), "max": v[-1]}


# Equal spacing: report i minus the mean of its neighbours has per-axis variance
# sigma^2 (1 + 1/4 + 1/4) = 1.5 sigma^2, so the 2-D error is Rayleigh with scale
# sigma sqrt(1.5), whose median is sqrt(2 ln 2) times the scale.
RAYLEIGH_MEDIAN = math.sqrt(2.0 * math.log(2.0)) * math.sqrt(1.5)
SIGMA_SPACING_S = 2.5


def estimate_sigma(stats: dict) -> float | None:
    """Declared estimator of the ADS-B horizontal 1-sigma from the tuning
    flights: the median leave-one-out error at neighbour spacing up to
    SIGMA_SPACING_S, divided by RAYLEIGH_MEDIAN, rounded to 0.1 m. Any motion
    the interpolant misses and any timing jitter inflate it, so it is an
    upper estimate of the noise."""
    q = stats[(0.0, SIGMA_SPACING_S)]
    return round(q["median"] / RAYLEIGH_MEDIAN, 1) if q["n"] else None


def choose_method(stats: dict[str, dict]) -> str:
    """Declared rule: the method with the lower p95 leave-one-out error on the
    tuning flights; ties go to the first of recorded.METHODS (hermite)."""
    best = None
    for m in recorded.METHODS:
        p = stats[m]["all"].get("p95", math.inf)
        if best is None or p < best[1]:
            best = (m, p)
    return best[0]


# ---- evaluation A: run directories -------------------------------------------------

@dataclass
class ARun:
    name: str
    split: str
    origin: tuple[float, float]
    members: list[tuple[FlightInfo, tuple[Report, ...]]] = field(default_factory=list)

    @property
    def start_ms(self) -> int:
        return min(c[0].t_ms for _, c in self.members)

    @property
    def end_ms(self) -> int:
        return max(c[-1].t_ms for _, c in self.members)


def a_runs(flights: list[FlightInfo]) -> list[ARun]:
    """Engine runs: flight chunks (realclean.chunks) of one split, merged into
    one run when their times overlap and all their reports lie within
    CHUNK_RADIUS_M of the run's origin, so aircraft that share airspace share
    a tracker."""
    pieces = sorted(((fi, c) for fi in flights for c in realclean.chunks(fi.flight)),
                    key=lambda p: (p[1][0].t_ms, p[0].name))
    runs: list[ARun] = []
    for fi, c in pieces:
        home = None
        for run in runs:
            if run.split != fi.split or c[0].t_ms > run.end_ms or c[-1].t_ms < run.start_ms:
                continue
            if all(geo.chord_m(run.origin[0], run.origin[1], r.lat, r.lon)
                   <= realclean.CHUNK_RADIUS_M for r in c):
                home = run
                break
        if home is None:
            home = ARun(f"{fi.split}_r{len(runs) + 1:03d}", fi.split, (c[0].lat, c[0].lon))
            runs.append(home)
        home.members.append((fi, c))
    return runs


def withheld_mask(reports: tuple[Report, ...], phase: int) -> list[bool]:
    """The declared schedule: after A_BURN_IN_S, gaps of A_GAPS_S (starting at
    index `phase`) separated by A_FEED_S of fed reports."""
    t0 = reports[0].t_ms
    gaps, cursor, i = [], t0 + A_BURN_IN_S * 1000, phase
    while cursor <= reports[-1].t_ms:
        g = A_GAPS_S[i % len(A_GAPS_S)] * 1000
        gaps.append((cursor, cursor + g))
        cursor += g + A_FEED_S * 1000
        i += 1
    out, j = [], 0
    for r in reports:
        while j < len(gaps) and r.t_ms >= gaps[j][1]:
            j += 1
        out.append(j < len(gaps) and gaps[j][0] <= r.t_ms < gaps[j][1])
    return out


@dataclass
class Fed:
    t_ns: int
    aircraft: int
    report: Report


def poll_feed(per_aircraft: list[tuple[int, list[Report], list[bool]]], interval_s: float,
              adapter: adsb.Adapter) -> list[tuple[wire.Measurement, int, Report]]:
    """What the live REST path would deliver: at each poll, every aircraft's
    latest report so far as an OpenSky state vector (time_position rounded
    down to whole seconds, as the API reports it), through the real adapter.
    A poll whose latest report is withheld sends nothing for that aircraft."""
    start = min(rs[0].t_ms for _, rs, _ in per_aircraft)
    end = max(rs[-1].t_ms for _, rs, _ in per_aircraft)
    out = []
    idx = {a: 0 for a, _, _ in per_aircraft}
    k = 0
    while True:
        poll_ms = start + round(k * interval_s * 1000)
        if poll_ms > end + interval_s * 1000:
            return out
        k += 1
        states, lookup = [], {}
        for a, rs, mask in per_aircraft:
            i = idx[a]
            while i + 1 < len(rs) and rs[i + 1].t_ms <= poll_ms:
                i += 1
            idx[a] = i
            r = rs[i]
            if r.t_ms > poll_ms or mask[i]:
                continue
            tpos = r.t_ms // 1000
            row = [r.icao24, None, None, tpos, tpos, r.lon, r.lat, r.alt_m, False,
                   r.speed_mps, r.track_deg, r.vrate_mps, None, r.alt_m, None, False, 0]
            states.append(row)
            lookup[(tpos * 1_000_000_000, r.lat, r.lon)] = (a, r)
        for m in adapter.convert({"time": poll_ms / 1000.0, "states": states}):
            a, r = lookup[(m.time_ns, m.z[0], m.z[1])]
            out.append((m, a, r))


def write_a_run(run: ARun, out_dir: pathlib.Path, feed: str, withhold: bool,
                sigma_m: float = SIGMA_M) -> Counter:
    """Writes one run directory for `feed` ("full" or "poll"); returns the
    adapter's drop counts (poll) and other feed statistics."""
    out_dir.mkdir(parents=True, exist_ok=True)
    stats: Counter = Counter()
    per_aircraft = []
    withheld = []
    for n, (fi, chunk) in enumerate(run.members):
        # Phase by position in the run (members are in start-time order), so the
        # schedule, like the frames, does not depend on the address.
        mask = withheld_mask(chunk, n % len(A_GAPS_S)) if withhold else [False] * len(chunk)
        per_aircraft.append((fi.aircraft, list(chunk), mask))
        withheld += [(r.t_ms * 1_000_000, fi.aircraft, r) for r, w in zip(chunk, mask) if w]
    stats["available"] = sum(len(rs) for _, rs, _ in per_aircraft)
    stats["withheld"] = len(withheld)
    if feed == "full":
        # Ties in time are broken by position, never by anything derived from
        # the address, so the tracker's input does not depend on it at all.
        fed = sorted(((r.t_ms * 1_000_000, a, r) for a, rs, mask in per_aircraft
                      for r, w in zip(rs, mask) if not w),
                     key=lambda x: (x[0], x[2].lat, x[2].lon, x[1]))
        frames = [(adsb.geo_measurement(ADSB_SENSOR_ID, i, t, r.lat, r.lon, r.alt_m, sigma_m), a, r)
                  for i, (t, a, r) in enumerate(fed)]
    elif feed == "poll":
        adapter = adsb.Adapter(ADSB_SENSOR_ID, sigma_m)
        frames = poll_feed(per_aircraft, POLL_INTERVAL_S, adapter)
        stats.update({f"adapter_{k}": v for k, v in adapter.dropped.items()})
    else:
        raise ValueError(feed)
    stats["fed"] = len(frames)
    (out_dir / "sensors.cfg").write_text(sensors.format_table(
        [sensors.SensorConfig(ADSB_SENSOR_ID, wire.GEO_POSITION, 0.0, 0.0, 0.0)]),
        encoding="ascii", newline="\n")
    (out_dir / "origin.txt").write_text(f"{run.origin[0]!r} {run.origin[1]!r} 0.0\n",
                                        encoding="ascii", newline="\n")
    (out_dir / "frames.hex").write_text("".join(wire.encode(m).hex() + "\n" for m, _, _ in frames),
                                        encoding="ascii", newline="\n")
    key = ["frame,aircraft,speed_mps,track_deg"]
    for i, (_, a, r) in enumerate(frames):
        v = "" if r.speed_mps is None else f"{r.speed_mps!r},{r.track_deg!r}"
        key.append(f"{i},{a},{v or ','}")
    (out_dir / "feed_key.csv").write_text("\n".join(key) + "\n", encoding="ascii", newline="\n")
    rows = ["time_ns,aircraft,lat,lon,alt_m"]
    for t, a, r in sorted(withheld, key=lambda x: (x[0], x[1])):
        rows.append(f"{t},{a},{r.lat!r},{r.lon!r},{0.0 if r.alt_m is None else r.alt_m!r}")
    (out_dir / "withheld.csv").write_text("\n".join(rows) + "\n", encoding="ascii", newline="\n")
    return stats


def run_tool(cmd: list[str]) -> None:
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        raise EvalError(f"{pathlib.Path(cmd[0]).name} failed ({p.returncode}): {p.stderr.strip()}")


def parse_realeval(text: str) -> dict:
    """The key-value lines of sifted_realeval."""
    out: dict = {"horizon": {}, "outcome": {}, "run": {}}
    for line in text.splitlines():
        if not line or line.startswith("#"):
            continue
        f = line.split()
        if f[0] == "horizon":
            n, med, p95, p99, mean = f[4:9]
            out["horizon"][(f[1], f[2], f[3])] = {"n": int(n), "median": float(med),
                                                  "p95": float(p95), "p99": float(p99),
                                                  "mean": float(mean)}
        elif f[0] == "outcome":
            out["outcome"][f[1]] = int(f[2])
        elif f[0] == "run":
            fields, i = {}, 2
            while i < len(f):  # "key value" pairs; coverage has two values
                if f[i] == "coverage":
                    fields["coverage"] = (int(f[i + 1]), int(f[i + 2]))
                    i += 3
                else:
                    fields[f[i]] = f[i + 1]
                    i += 2
            out["run"][f[1]] = fields
        elif f[0] == "nis":
            out["nis"] = {"n": int(f[1]), "mean": float(f[2]), "expected": float(f[3]),
                          "lo": float(f[4]), "hi": float(f[5])}
        elif f[0] == "coverage":
            out["coverage"] = (int(f[1]), int(f[2]))
        else:
            out[f[0]] = int(f[1])
    return out


# Feeds of evaluation A: (directory, feed, withhold, engine args, sigma).
A_CASES = (
    ("all", "full", False, "adsb", "assumed"),         # every report: continuity and NIS
    ("withheld", "full", True, "adsb", "assumed"),     # prediction at withheld reports
    ("poll_all", "poll", False, "adsb", "assumed"),    # live REST cadence
    ("poll_withheld", "poll", True, "adsb", "assumed"),
    ("all_default", "full", False, "default", "assumed"),  # same input, EngineConfig defaults
    # Diagnostic only (no default changes): the sigma estimated on the tuning flights.
    ("all_sigma", "full", False, "adsb", "estimated"),
    ("withheld_sigma", "full", True, "adsb", "estimated"),
)


def evaluation_a(flights: list[FlightInfo], work: pathlib.Path, bin_dir: pathlib.Path,
                 sigma_est: float | None) -> dict:
    runs = a_runs(flights)
    tool = str(bin_dir / "sifted_realeval")
    results: dict = {"runs": runs, "cases": {}, "feed_stats": {}}
    for split in SPLITS:
        split_runs = [r for r in runs if r.split == split]
        if not split_runs:
            continue
        for case, feed, withhold, cfg, sigma in A_CASES:
            if sigma == "estimated" and sigma_est is None:
                continue
            root = work / "A" / split / case
            stats: Counter = Counter()
            if cfg == "default":
                root = work / "A" / split / "all"  # same frames as "all"
            else:
                if root.exists():
                    shutil.rmtree(root)
                for run in split_runs:
                    stats += write_a_run(run, root / run.name, feed, withhold,
                                         SIGMA_M if sigma == "assumed" else sigma_est)
            out = work / "A" / f"{split}_{case}.txt"
            run_tool([tool, "--runs", str(root), "--bins", ",".join(map(str, HORIZON_EDGES_S)),
                      *(ADSB_TRACKER_ARGS if cfg == "adsb" else []), "--out", str(out)])
            results["cases"][(split, case)] = parse_realeval(out.read_text(encoding="ascii"))
            results["feed_stats"][(split, case)] = stats
    return results


# ---- evaluation B: scenario directories --------------------------------------------

@dataclass
class Window:
    name: str
    flight: FlightInfo
    start_ms: int
    turn_deg: float


def heading_change_deg(knots: list[recorded.Knot]) -> float:
    """Range (max - min) of the unwrapped reported heading over the knots [deg].

    A range rather than a sum of absolute steps, so report-to-report heading
    jitter does not accumulate into an apparent turn."""
    lo = hi = cur = 0.0
    prev = None
    for k in knots:
        if k.vx is None or math.hypot(k.vx, k.vy) < 1.0:
            continue
        h = math.atan2(k.vy, k.vx)
        if prev is not None:
            cur += math.remainder(h - prev, 2 * math.pi)
            lo, hi = min(lo, cur), max(hi, cur)
        prev = h
    return math.degrees(hi - lo)


def b_windows(fi: FlightInfo, method: str, skipped: Counter):
    """Windows of one flight, each with its scenarios (real and control) per seed."""
    rs = fi.flight.reports
    w_ms = int(B_WINDOW_S * 1000)
    k = 0
    while fi.flight.start_ms + (k + 1) * w_ms <= fi.flight.end_ms:
        ws = fi.flight.start_ms + k * w_ms
        we = ws + w_ms
        k += 1
        inside = [r for r in rs if ws - B_KNOT_MARGIN_S * 1000 <= r.t_ms <= we + B_KNOT_MARGIN_S * 1000]
        if inside[0].t_ms > ws or inside[-1].t_ms < we:
            skipped["window_not_covered"] += 1
            continue
        if max(b.t_ms - a.t_ms for a, b in zip(inside, inside[1:])) > B_MAX_KNOT_GAP_S * 1000:
            skipped["window_knot_gap"] += 1
            continue
        mid = min(inside, key=lambda r: (abs(r.t_ms - (ws + we) // 2), r.t_ms))
        enu = geo.Enu(mid.lat, mid.lon)
        knots = knots_of(inside, enu, inside[0].t_ms)
        traj = recorded.RecordedTrajectory(knots, method)
        start_s = (inside[0].t_ms - ws) / 1000.0  # <= 0: the path starts before the window
        target = Target(1, traj, start_s)
        p0, pm, pw = target.state_at(0.0), target.state_at(B_WINDOW_S / 2), target.state_at(B_WINDOW_S)
        h = math.atan2(pm.vy_mps, pm.vx_mps)
        nx, ny = -math.sin(h), math.cos(h)
        window_knots = [kn for kn, r in zip(knots, inside) if ws <= r.t_ms <= we]
        win = Window(f"{fi.name}_w{k - 1:03d}", fi, ws, heading_change_deg(window_knots))
        control = Target(1, Trajectory(TargetState(p0.x_m, p0.y_m, (pw.x_m - p0.x_m) / B_WINDOW_S,
                                                   (pw.y_m - p0.y_m) / B_WINDOW_S),
                                       [Straight(B_WINDOW_S)]))
        for seed in B_SEEDS:
            base = (int(fi.name[1:]) * 1000 + k) * 10 + seed
            radar = RadarParams(sensor_id=1, x_m=pm.x_m + B_SENSOR_OFFSET_M * nx,
                                y_m=pm.y_m + B_SENSOR_OFFSET_M * ny, yaw_rad=0.0,
                                sigma_range_m=SIGMA_RANGE_M, sigma_bearing_rad=SIGMA_BEARING_RAD,
                                rate_hz=RATE_HZ, seed=base, max_range_m=B_RADAR_RANGE_M,
                                p_detect=0.9, clutter_per_scan=0.5)
            camera = CameraParams(sensor_id=2, x_m=pm.x_m - B_SENSOR_OFFSET_M * nx,
                                  y_m=pm.y_m - B_SENSOR_OFFSET_M * ny, yaw_rad=0.0,
                                  sigma_bearing_rad=math.radians(0.1), rate_hz=2.0,
                                  seed=base + 5, max_range_m=B_CAMERA_RANGE_M, p_detect=0.95,
                                  phase_s=0.25)
            for variant, tgt in (("real", target), ("cv", control)):
                yield win, seed, variant, MultiScenario(f"{win.name}_s{seed}", [radar], [camera],
                                                        [tgt], B_WINDOW_S, start_ns=ws * 1_000_000)


def evaluation_b(flights: list[FlightInfo], method: str, work: pathlib.Path,
                 bin_dir: pathlib.Path) -> dict:
    tool = str(bin_dir / "sifted_eval")
    skipped: Counter = Counter()
    windows: dict[str, Window] = {}
    for split in SPLITS:
        for variant in ("real", "cv"):
            d = work / "B" / split / variant
            if d.exists():
                shutil.rmtree(d)
    count: Counter = Counter()
    for fi in flights:
        for win, seed, variant, sc in b_windows(fi, method, skipped):
            windows[win.name] = win
            generate.write(sc, work / "B" / fi.split / variant / sc.name)
            count[(fi.split, variant)] += 1
    rows: dict = {}
    for split in SPLITS:
        for variant in ("real", "cv"):
            if not count[(split, variant)]:
                continue
            out = work / "B" / f"{split}_{variant}.csv"
            run_tool([tool, "--scenarios", str(work / "B" / split / variant), "--levels", "clean",
                      "--seeds", "1", "--format", "csv", "--match-distance",
                      repr(B_MATCH_DISTANCE_M), "--out", str(out)])
            with open(out, encoding="utf-8", newline="") as f:
                for row in csv.DictReader(f):
                    if row["scenario"] != "ALL" and row["condition"] == "direct":
                        rows[(split, variant, row["scenario"])] = row
    return {"windows": windows, "rows": rows, "skipped": skipped}


def b_summary(b: dict, split: str, turning: bool | None = None) -> dict:
    """Aggregates evaluation B over the windows of one split (optionally only
    manoeuvring or only straight windows): per-run RMSE quantiles, the error
    budget against the constant-velocity control, and seed vs window spread."""
    def ok(name: str) -> bool:
        w = b["windows"][name.rsplit("_s", 1)[0]]
        return w.flight.split == split and (turning is None or (w.turn_deg > B_TURN_DEG) == turning)

    out = {}
    for variant in ("real", "cv"):
        rs = {n: r for (s, v, n), r in b["rows"].items() if s == split and v == variant and ok(n)}
        rmse = {n: float(r["mean_rmse_m"]) for n, r in rs.items() if r["mean_rmse_m"]}
        out[variant] = {
            "runs": len(rs), "scored": len(rmse), "rmse": quant(list(rmse.values())),
            "rms": math.sqrt(math.fsum(v * v for v in rmse.values()) / len(rmse)) if rmse else None,
            "missed": sum(int(r["missed"]) for r in rs.values()),
            "eligible": sum(int(r["eligible"]) for r in rs.values()),
            "id_switches": sum(int(r["id_switches"]) for r in rs.values()),
            "false_tracks": sum(int(r["false_tracks"]) for r in rs.values()),
            "nees2": _mean([r["nees2_updated"] for r in rs.values()]),
            "nis_rb": _mean([r["nis_range_bearing"] for r in rs.values()]),
            "per_run": rmse,
        }
    real, cv = out["real"]["per_run"], out["cv"]["per_run"]
    both = sorted(set(real) & set(cv))
    if both:
        r2 = math.fsum(real[n] ** 2 for n in both) / len(both)
        c2 = math.fsum(cv[n] ** 2 for n in both) / len(both)
        out["budget"] = {"pairs": len(both), "real_rms": math.sqrt(r2), "cv_rms": math.sqrt(c2),
                         "motion_share": max(0.0, 1.0 - c2 / r2) if r2 > 0 else 0.0,
                         "excess_rms": math.sqrt(max(0.0, r2 - c2))}
        by_window: dict[str, list[float]] = {}
        for n in sorted(real):
            by_window.setdefault(n.rsplit("_s", 1)[0], []).append(real[n])
        multi = [v for v in by_window.values() if len(v) > 1]
        if multi:
            out["spread"] = {
                "seed_sd": statistics.fmean(statistics.pstdev(v) for v in multi),
                "window_sd": statistics.pstdev([statistics.fmean(v) for v in multi]),
            }
    return out


def _mean(vals: list[str]) -> float | None:
    v = [float(x) for x in vals if x]
    return math.fsum(v) / len(v) if v else None


# ---- report ------------------------------------------------------------------------

def f1(v) -> str:
    return "–" if v is None else f"{v:.1f}"


def f2(v) -> str:
    return "–" if v is None else f"{v:.2f}"


def pct(n: int, d: int) -> str:
    return "–" if d == 0 else f"{100.0 * n / d:.1f}%"


def render(manifest: dict, datasets: list[Dataset], flights: list[FlightInfo], removed: dict,
           removed_total: Counter, loo_tune: dict, loo_held: dict, method: str, a: dict, b: dict,
           command: str) -> str:
    synthetic = bool(manifest.get("synthetic"))
    L: list[str] = []
    add = L.append
    add("# Real-data evaluation\n")
    add("<!-- Generated by sim/sifted_sim/realeval.py with eval/sifted_realeval.cpp and "
        "eval/sifted_eval.cpp. Do not edit by hand. -->\n")
    if synthetic:
        add("> **SYNTHETIC FIXTURE.** These numbers come from a generated test input, not from "
            "real aircraft. They exercise the pipeline and mean nothing about tracking accuracy.\n")
    add("How Sifted behaves on recorded aircraft data, as opposed to its own simulator "
        "([evaluation.md](evaluation.md)), whose noise matches the filter's assumptions exactly "
        "and whose targets fly straight legs and constant-rate turns. Two complementary "
        "evaluations, on the same cleaned flights:\n")
    add("- **A, real measurements.** Real ADS-B positions through the real wire codec, decoder "
        "and engine. With no independent truth, it is scored predictively: runs of each "
        "aircraft's reports are withheld, and at each withheld report the tracker's estimate is "
        "compared with it, against two baselines (holding the last report; dead-reckoning it by "
        "its reported velocity).")
    add("- **B, real trajectories with simulated sensors.** The flight paths, interpolated, are "
        "the truth for the simulator's seeded radar and camera, scored by the same `sifted_eval` "
        "as the simulator scorecard; a constant-velocity control with identical sensors, noise "
        "and seeds separates the error caused by real motion from the error caused by noise.\n")
    add("All numbers on this page come from one command, run from the repository root on the "
        "cached data (download once with `python3 -m sifted_sim.realeval fetch`):\n")
    add("```bash\ncmake --preset release && cmake --build --preset release\n"
        f"cd sim && {command}\n```\n")
    add("The page is a deterministic function of the cached data and the binaries: running the "
        "command twice gives a byte-identical file.\n")

    # Headline.
    held = a["cases"].get(("heldout", "withheld"))
    hall = a["cases"].get(("heldout", "all"))
    bs = b_summary(b, "heldout") if b["rows"] else None
    add("## Summary (held-out flights)\n")
    if held and hall:
        def hz(est, lo, hi):
            return held["horizon"].get((est, lo, hi), {})
        t5, d5 = hz("tracker", "2", "5"), hz("dead_reckoning", "2", "5")
        t40, d40, h40 = hz("tracker", "20", "40"), hz("dead_reckoning", "20", "40"), hz("hold", "20", "40")
        t80, d80 = hz("tracker", "40", "80"), hz("dead_reckoning", "40", "80")
        add(f"- **Prediction (A).** Median error of the tracker's estimate at a withheld report "
            f"{f1(t5.get('median'))} m at 2–5 s horizons (dead reckoning {f1(d5.get('median'))} m), "
            f"{f1(t40.get('median'))} m at 20–40 s (dead reckoning {f1(d40.get('median'))} m, "
            f"holding the last report {f1(h40.get('median'))} m) and {f1(t80.get('median'))} m at "
            f"40–80 s (dead reckoning {f1(d80.get('median'))} m).")
        nis = hall.get("nis", {})
        add(f"- **Consistency (A).** Average NIS of real ADS-B updates {f2(nis.get('mean'))} "
            f"(2.00 expected for a consistent filter; 95% interval under independence "
            f"[{f2(nis.get('lo'))}, {f2(nis.get('hi'))}]) with the assumed {SIGMA_M:.0f} m "
            f"ADS-B sigma: {nis_verdict(nis)}.")
        cov = hall.get("coverage", (0, 0))
        pa = a["cases"].get(("heldout", "poll_all"))
        live = (f" At the live REST cadence ({POLL_INTERVAL_S:.1f} s polls, "
                f"{pct(pa['fed'], hall['fed'])} of the reports): {pa['fragments']} "
                f"fragmentations, coverage {pct(pa['coverage'][1], pa['coverage'][0])}." if pa else "")
        add(f"- **Continuity (A).** {hall['runs']} held-out engine runs ({hall['aircraft']} "
            f"aircraft): {hall['confirmed_tracks']} confirmed tracks, "
            f"{hall['fragments']} fragmentations, {hall['swaps']} identity swaps (none possible: "
            f"one aircraft per run), coverage {pct(cov[1], cov[0])} of reports after "
            f"confirmation.{live}")
    if bs and "budget" in bs:
        bb = bs["budget"]
        add(f"- **Real motion, simulated sensors (B).** Position RMSE per window: median "
            f"{f1(bs['real']['rmse'].get('median'))} m, p95 {f1(bs['real']['rmse'].get('p95'))} m, "
            f"against {f1(bs['cv']['rmse'].get('median'))} m / {f1(bs['cv']['rmse'].get('p95'))} m "
            f"for the same sensors on a constant-velocity path; real motion accounts for "
            f"{100 * bb['motion_share']:.0f}% of the mean squared error. "
            f"Truth quality limits this: leave-one-out interpolation error p95 "
            f"{f1(loo_held[method]['all'].get('p95'))} m.")
    add("")
    add("Unfavourable findings are listed under [Findings](#findings) and "
        "[Limitations](#limitations).\n")

    # Provenance.
    add("## Provenance\n")
    terms = manifest.get("terms", "")
    if terms:
        add(terms + "\n")
    add("| dataset | archive | SHA-256 | retrieved | state vectors | valid reports | aircraft | "
        "time range (UTC) | area (lat, lon) | licence |")
    add("|---|---|---|---|---:|---:|---:|---|---|---|")
    for ds in datasets:
        e = ds.entry
        got = describe(ds.reports, ds.rows)
        tr = got.get("time_range_utc", ["–", "–"])
        ar = got.get("area_deg")
        area = (f"{ar['lat_min']:.2f}..{ar['lat_max']:.2f}, {ar['lon_min']:.2f}..{ar['lon_max']:.2f}"
                if ar else "–")
        src = f"[{e['file']}]({e['url']})" if e.get("url") else f"`{e['file']}`"
        add(f"| {ds.label} | {src} | `{e['sha256'][:16]}…` | {e.get('retrieved', '–')} | "
            f"{ds.rows} | {len(ds.reports)} | {got['aircraft']} | {tr[0]} – {tr[1]} | {area} | "
            f"{e.get('licence', '–')} |")
    add("")
    add("Aircraft addresses, callsigns and registrations appear nowhere in the results: aircraft "
        "are numbered labels and flights are numbered by start time. The tracker never receives "
        "an address (`test_tracker_input_carries_no_identity` in `sim/tests/test_realeval.py`).\n")

    # Cleaning.
    add("## Cleaning\n")
    add("Reader checks (`sim/sifted_sim/statevectors.py`, the untrusted-input boundary) and then "
        "cleaning rules (`sim/sifted_sim/realclean.py`), applied to every aircraft alike. Each "
        "report is counted once, under the first rule that removes it.\n")
    reasons_read = sorted({k for ds in datasets for k in ds.read_drops})
    reasons_clean = sorted({k for c in removed.values() for k, v in c.items() if v})
    add("| dataset | rows | " + " | ".join(f"read: {r}" for r in reasons_read) + " | "
        + " | ".join(reasons_clean) + " | kept | flights |")
    add("|---|---:|" + "---:|" * (len(reasons_read) + len(reasons_clean) + 2))
    for ds in datasets:
        kept = sum(len(fi.flight.reports) for fi in flights if fi.flight.dataset == ds.label)
        nf = sum(1 for fi in flights if fi.flight.dataset == ds.label)
        add(f"| {ds.label} | {ds.rows} | " + " | ".join(str(ds.read_drops.get(r, 0)) for r in reasons_read)
            + " | " + " | ".join(str(removed[ds.label].get(r, 0)) for r in reasons_clean)
            + f" | {kept} | {nf} |")
    tot_rows = sum(ds.rows for ds in datasets)
    kept_all = sum(len(fi.flight.reports) for fi in flights)
    add(f"\nIn all: {tot_rows} state vectors, {kept_all} reports kept in {len(flights)} flights "
        f"({pct(kept_all, tot_rows)}). Most removals are `duplicate_time`: a state vector "
        "re-emitted with a new velocity but no new position. Reports older than one before them "
        "in file order (what an arrival-order feed would drop under the engine's clock rule): "
        + ", ".join(f"{ds.label} {ds.file_order_late}" for ds in datasets) + ".\n")
    add("Parameters: " + ", ".join(f"`{k}` = {v:g}" for k, v in realclean.PARAMETERS.items()) + ".\n")
    add("| flight | split | aircraft | duration s | reports | median spacing s | max gap s |")
    add("|---|---|---:|---:|---:|---:|---:|")
    for fi in flights:
        rs = fi.flight.reports
        sp = sorted((b.t_ms - a.t_ms) / 1000 for a, b in zip(rs, rs[1:]))
        add(f"| {fi.name} | {fi.split} | {fi.aircraft} | {fi.flight.duration_s:.0f} | {len(rs)} | "
            f"{sp[len(sp) // 2]:.1f} | {sp[-1]:.1f} |")
    add("")

    # Method.
    add("## Method\n")
    add(f"**Selection.** Every flight that survives cleaning is used; none is chosen by hand. "
        f"Flights are numbered by start time and every {TUNING_EVERY}rd (F01, F04, ...) forms the "
        "tuning portion; the rest are held out. No tracker parameter is tuned at all; the two "
        "choices made from data, the interpolation method of evaluation B and the diagnostic "
        "sigma of evaluation A, are made on the tuning flights only. Headline numbers are "
        "held-out.\n")
    add(f"**Evaluation A.** Engine runs are flight chunks whose reports all lie within "
        f"{realclean.CHUNK_RADIUS_M / 1000:.0f} km of the run's tangent-plane origin (its first "
        f"report); chunks that overlap in time and fit within that radius share a run. The "
        f"furthest report from an origin in these data is "
        f"{max(c.get('max_range_m', 0) for c in a['cases'].values()) / 1000:.0f} km; at that "
        "range the flat-plane distortion of a straight constant-speed path is below 0.1% of its "
        "speed, far under the measurement noise. Engine configuration: the launcher's ADS-B "
        f"settings (`{' '.join(ADSB_TRACKER_ARGS)}`; `app/launcher.py` `ADSB_TRACKER_ARGS`), "
        f"assumed sigma {SIGMA_M:.0f} m (`sifted_sim.adsb`). The estimator is the "
        "`EngineConfig` default, the IMM (ADR 0014); evaluation B's `sifted_eval` runs use the "
        "same default. Feeds:\n")
    add("- *all*: every cleaned report, at its millisecond position time (`lastposupdate`), "
        "nothing withheld. Continuity and NIS come from this feed.")
    add(f"- *withheld*: the same, minus the withheld reports: after {A_BURN_IN_S:.0f} s, gaps of "
        f"{', '.join(f'{g:g}' for g in A_GAPS_S)} s in turn, each followed by {A_FEED_S:.0f} s "
        "of fed reports. Prediction errors come from this feed.")
    add(f"- *poll* / *poll withheld*: the live REST path. Every {POLL_INTERVAL_S:.1f} s (the "
        "signed-in quota floor for a 1-credit box) each aircraft's latest report becomes an "
        "OpenSky state vector with whole-second `time_position` and goes through the real "
        "adapter (`sifted_sim.adsb.Adapter`), which drops repeats and stale or out-of-order "
        "positions.")
    add("- *all, default config*: the *all* frames with `EngineConfig` defaults (lifecycle "
        "sized for 1 Hz sensors), to show why the ADS-B settings exist.")
    add(f"- *sigma {f1(a.get('sigma_est'))} m* (diagnostic): *all* and *withheld* with the "
        "assumed sigma replaced by one estimated on the tuning flights: the median leave-one-out "
        f"error at spacing up to {SIGMA_SPACING_S:g} s divided by {RAYLEIGH_MEDIAN:.3f} (the "
        "median of the Rayleigh distance of a report from the mean of its two neighbours, per "
        "unit sigma). Motion the interpolant misses and timing jitter inflate it, so it is an "
        "upper estimate. Nothing in the tracker or adapter defaults changes.\n")
    add("Horizon = withheld report time minus the aircraft's last fed report. The tracker's "
        "estimate is the operator picture at that time (`tracks_if_silent_until`): the track the "
        "aircraft's last fed report went to, if confirmed or coasting; otherwise it counts as "
        "*no track*. The aircraft label is used only to pick that track and to group results. "
        "Quantiles are nearest-rank. Definitions: `eval/sifted/eval/realdata.hpp`.\n")
    add(f"**Evaluation B.** Each flight is cut into consecutive {B_WINDOW_S:.0f} s windows; a "
        f"window is used if reports cover it with no gap over {B_MAX_KNOT_GAP_S:.0f} s "
        f"(skipped: {', '.join(f'{v} {k}' for k, v in sorted(b['skipped'].items())) or 'none'}). "
        f"The path is interpolated by **{method}** (chosen on the tuning flights, below). A radar (the simulator's: "
        f"{SIGMA_RANGE_M:.0f} m, {math.degrees(SIGMA_BEARING_RAD):.1f} deg, {RATE_HZ:.0f} Hz, "
        f"Pd 0.9, 0.5 clutter/scan) with {B_RADAR_RANGE_M / 1000:.0f} km coverage sits "
        f"{B_SENSOR_OFFSET_M / 1000:.0f} km left of the track at the window midpoint, and a "
        f"camera (0.1 deg, 2 Hz) as far to the right; seeds {', '.join(map(str, B_SEEDS))}. The "
        "control replaces the path by the straight line from its start to its end position over "
        "the same time. Scored by `sifted_eval --levels clean --match-distance "
        f"{B_MATCH_DISTANCE_M:.0f}` (direct engine feed, default `EngineConfig`): the simulator "
        "scorecard's 100 m match distance is sized for an 8 km radar, and the same rule (about "
        "four times the raw radar error at the coverage edge, 71 m here) gives "
        f"{B_MATCH_DISTANCE_M:.0f} m. Windows whose reported heading ranges over more than "
        f"{B_TURN_DEG:.0f} deg count as manoeuvring.\n")

    # Interpolation.
    add("## Truth quality: interpolation (leave-one-out)\n")
    add("Each interior report is removed and predicted from its two neighbours; the error mixes "
        "interpolation error over twice the real spacing with the report's own noise, so it "
        "bounds the truth error of evaluation B from above. Median / p95 / max in metres, by "
        "neighbour spacing.\n")
    add("| portion | method | n | all | " + " | ".join(f"{lo:g}–{hi:g} s" for lo, hi in
                                                    zip(LOO_EDGES_S, LOO_EDGES_S[1:])) + " |")
    add("|---|---|---:|---|" + "---|" * (len(LOO_EDGES_S) - 1))
    for portion, st in (("tuning", loo_tune), ("held-out", loo_held)):
        for m in recorded.METHODS:
            cells = [st[m]["all"]] + [st[m][(lo, hi)] for lo, hi in zip(LOO_EDGES_S, LOO_EDGES_S[1:])]
            add(f"| {portion} | {m} | {st[m]['all']['n']} | " + " | ".join(
                f"{f1(c.get('median'))} / {f1(c.get('p95'))} / {f1(c.get('max'))}" if c["n"] else "–"
                for c in cells) + " |")
    add(f"\nChosen on the tuning flights (lower p95): **{method}**. The two methods are nearly "
        "equal at the ~2 s spacing that dominates: at that spacing the error is mostly the "
        "reports' own scatter, not the interpolant.\n")

    # Results A.
    add("## Results A: real measurements\n")
    for split in ("heldout", "tuning"):
        w = a["cases"].get((split, "withheld"))
        pw = a["cases"].get((split, "poll_withheld"))
        if not w:
            continue
        title = "held-out" if split == "heldout" else "tuning (for comparison)"
        add(f"### Prediction error at withheld reports, {title}\n")
        add("Metres: n, median / p95 / p99. *no track*: withheld reports at which the tracker "
            "offered no confirmed or coasting track for the aircraft.\n")
        add("| feed | horizon s | tracker | dead reckoning | hold last report |")
        add("|---|---|---|---|---|")
        ws = a["cases"].get((split, "withheld_sigma"))
        for label, case in (("withheld", w), ("poll withheld", pw),
                            (f"withheld, sigma {f1(a.get('sigma_est'))} m", ws)):
            if not case:
                continue
            edges = [str(e) for e in HORIZON_EDGES_S] + ["inf"]
            for lo, hi in [("all", "all")] + list(zip(edges, edges[1:])):
                cells = []
                for est in ("tracker", "dead_reckoning", "hold"):
                    c = case["horizon"].get((est, lo, hi), {"n": 0})
                    cells.append(f"{c['n']}: {f1(c['median'])} / {f1(c['p95'])} / {f1(c['p99'])}"
                                 if c["n"] else "–")
                if all(c == "–" for c in cells):
                    continue
                hl = "all" if lo == "all" else f"{lo}–{hi}"
                add(f"| {label} | {hl} | " + " | ".join(cells) + " |")
            add(f"| {label} | no track | {case['no_track']} of {case['withheld']} withheld | "
                f"{case['no_velocity']} without velocity | – |")
        add("")
    add("### Consistency and continuity\n")
    add("NIS: average normalised innovation squared of geodetic updates after each track's "
        "first 10 (2.00 expected). Coverage: share of an aircraft's reports, after its first "
        "three in a run, that updated a confirmed track.\n")
    add("| portion | feed | runs | aircraft | reports fed | tracks started | confirmed | "
        "fragmentations | never confirmed | swaps | mixed | inconsistent | coverage | NIS (n) | "
        "engine out-of-order |")
    add("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|")
    for split in ("heldout", "tuning"):
        for case, label in (("all", "all"), ("poll_all", "poll"), ("all_default", "all, default config"),
                            ("all_sigma", f"all, sigma {f1(a.get('sigma_est'))} m")):
            c = a["cases"].get((split, case))
            if not c:
                continue
            nis = c.get("nis", {})
            add(f"| {split} | {label} | {c['runs']} | {c['aircraft']} | {c['fed']} | "
                f"{c['tracks_started']} | {c['confirmed_tracks']} | {c['fragments']} | "
                f"{c['never_confirmed']} | {c['swaps']} | {c['mixed_tracks']} | "
                f"{c['inconsistent_tracks']} | {pct(c['coverage'][1], c['coverage'][0])} | "
                f"{f2(nis.get('mean'))} ({nis.get('n', 0)}) | {c['outcome'].get('out_of_order', 0)} |")
    add("")
    add("Per run (held-out, feed *all*):\n")
    add("| run | flights | reports | median speed m/s | tracks started | confirmed | "
        "fragmentations | coverage | max range km | default config: started / confirmed / coverage |")
    add("|---|---|---:|---:|---:|---:|---:|---:|---:|---|")
    members = {r.name: r for r in a["runs"]}
    dflt_runs = a["cases"].get(("heldout", "all_default"), {}).get("run", {})
    for name, r in sorted(a["cases"].get(("heldout", "all"), {}).get("run", {}).items()):
        fl = ", ".join(sorted({fi.name for fi, _ in members[name].members}))
        d = dflt_runs.get(name)
        dc = f"{d['started']} / {d['confirmed']} / {pct(d['coverage'][1], d['coverage'][0])}" if d else "–"
        add(f"| {name} | {fl} | {r['fed']} | {f1(run_speed(members[name]))} | {r['started']} | "
            f"{r['confirmed']} | {r['fragments']} | {pct(r['coverage'][1], r['coverage'][0])} | "
            f"{int(r['max_range_m']) / 1000:.0f} | {dc} |")
    add("")
    add("### What the live REST path delivers\n")
    add("| portion | reports available | delivered by polling | " + " | ".join(
        sorted({k[8:] for st in a["feed_stats"].values() for k in st if k.startswith("adapter_")}))
        + " |")
    keys = sorted({k for st in a["feed_stats"].values() for k in st if k.startswith("adapter_")})
    add("|---|---:|---:|" + "---:|" * len(keys))
    for split in ("heldout", "tuning"):
        st = a["feed_stats"].get((split, "poll_all"))
        if st:
            add(f"| {split} | {st['available']} | {st['fed']} ({pct(st['fed'], st['available'])}) | "
                + " | ".join(str(st.get(k, 0)) for k in keys) + " |")
    add(f"\nAt a {POLL_INTERVAL_S:.1f} s poll the tracker sees one report per aircraft per poll at "
        "most; the adapter's `repeat` drops are polls at which the aircraft had no newer "
        "position, `stale` ones are positions older than its 15 s limit. Its `older_than_sent` "
        "rule (and the engine's out-of-order rule) can only bite when aircraft with interleaved "
        "times share a response: each archive here holds one aircraft, so it did not.\n")

    # Results B.
    add("## Results B: real trajectories, simulated sensors\n")
    add("Per run (window x seed) position RMSE in metres over the scored instants (after the "
        "10 s burn-in), direct engine feed: n, median / p95 / max. Missed: covered target-instants "
        "unmatched within the match distance.\n")
    add("| portion | windows | path | runs scored | RMSE median / p95 / max | RMS | missed | "
        "ID sw. | false | NEES2 upd. | NIS radar |")
    add("|---|---|---|---:|---|---:|---:|---:|---:|---:|---:|")
    for split in ("heldout", "tuning"):
        for turning, wl in ((None, "all"), (False, "straight"), (True, "manoeuvring")):
            s = b_summary(b, split, turning) if b["rows"] else None
            if not s or not s["real"]["runs"]:
                continue
            for variant, vl in (("real", "real path"), ("cv", "control")):
                v = s[variant]
                q = v["rmse"]
                add(f"| {split} | {wl} | {vl} | {v['scored']}/{v['runs']} | "
                    f"{f1(q.get('median'))} / {f1(q.get('p95'))} / {f1(q.get('max'))} | "
                    f"{f1(v['rms'])} | {pct(v['missed'], v['eligible'])} | {v['id_switches']} | "
                    f"{v['false_tracks']} | {f2(v['nees2'])} | {f2(v['nis_rb'])} |")
    add("")
    add("| portion | windows | pairs | RMS real / control m | share of MSE from real motion | "
        "excess RMS m | seed-to-seed SD m | window-to-window SD m |")
    add("|---|---|---:|---|---:|---:|---:|---:|")
    for split in ("heldout", "tuning"):
        for turning, wl in ((None, "all"), (False, "straight"), (True, "manoeuvring")):
            s = b_summary(b, split, turning) if b["rows"] else None
            if not s or "budget" not in s:
                continue
            bb, sp = s["budget"], s.get("spread", {})
            add(f"| {split} | {wl} | {bb['pairs']} | {f1(bb['real_rms'])} / {f1(bb['cv_rms'])} | "
                f"{100 * bb['motion_share']:.0f}% | {f1(bb['excess_rms'])} | "
                f"{f1(sp.get('seed_sd'))} | {f1(sp.get('window_sd'))} |")
    add(f"\nSeed-to-seed SD: spread of a window's RMSE over the {len(B_SEEDS)} noise seeds "
        "(noise); window-to-window SD: spread of the seed-averaged RMSE between windows (mostly "
        "motion and geometry). Interpolation bound for these numbers: leave-one-out p95 "
        f"{f1(loo_held[method]['all'].get('p95'))} m (held-out, {method}).\n")

    add(findings(a, b, loo_held, method))
    add(limitations(a, flights, synthetic))
    add("## Parameters\n")
    add("| parameter | value |")
    add("|---|---|")
    for k, v in sorted({**PARAMETERS, **realclean.PARAMETERS}.items()):
        add(f"| `{k}` | {', '.join(f'{x:g}' for x in v) if isinstance(v, tuple) else f'{v:g}'} |")
    add("")
    add("## Reproduction\n")
    add("```bash\ncmake --preset release && cmake --build --preset release\n"
        "cd sim && python3 -m sifted_sim.realeval fetch   # once; verifies every SHA-256\n"
        f"{command}\n```\n")
    add("Tests (offline, synthetic fixtures): `sim/tests/test_realeval.py` (parser, cleaning, "
        "interpolation, withholding, identity, end-to-end determinism when the binaries are "
        "built), `sim/tests/test_statevectors_fuzz.py` (property tests of the reader) and "
        "`Eval.*` in `tests/eval_test.cpp` (the scorecard is unchanged by the new scoring "
        "parameters).\n")
    return "\n".join(L).rstrip("\n") + "\n"


def run_speed(run: ARun) -> float | None:
    v = sorted(r.speed_mps for _, c in run.members for r in c if r.speed_mps is not None)
    return v[len(v) // 2] if v else None


def nis_verdict(nis: dict) -> str:
    if not nis.get("n"):
        return "no samples"
    mean = nis["mean"]
    if nis["lo"] <= mean <= nis["hi"]:
        return "consistent"
    ratio = math.sqrt(mean / nis["expected"])
    return f"not consistent, innovations {ratio:.2f}x the size the filter predicts"


def findings(a: dict, b: dict, loo_held: dict, method: str) -> str:
    """Findings stated from the numbers (the text adapts to what was measured)."""
    out = ["## Findings\n"]
    hall = a["cases"].get(("heldout", "all"))
    held = a["cases"].get(("heldout", "withheld"))
    dflt = a["cases"].get(("heldout", "all_default"))
    if hall:
        nis = hall.get("nis", {})
        if nis.get("n"):
            out.append(f"- **ADS-B noise assumption.** NIS {f2(nis['mean'])} against 2.00: "
                       f"{nis_verdict(nis)}, so the filter believes its innovations are "
                       "larger than they are. The innovation covers the measurement noise and "
                       "the filter's prediction error; the leave-one-out scatter of the reports "
                       f"themselves (held-out median {f1(loo_held[method]['all'].get('median'))} m "
                       f"at ~2 s spacing) is far below the assumed {SIGMA_M:.0f} m 1-sigma. The "
                       "assumed value is left as it is (results here are not used to change "
                       "defaults); see the diagnostic below for what a measured value does.")
    if held:
        rows = []
        for lo, hi in zip([str(e) for e in HORIZON_EDGES_S], [str(e) for e in HORIZON_EDGES_S[1:]] + ["inf"]):
            t = held["horizon"].get(("tracker", lo, hi), {"n": 0})
            d = held["horizon"].get(("dead_reckoning", lo, hi), {"n": 0})
            if t["n"] and d["n"]:
                rows.append((lo, hi, t["median"], d["median"], t["n"]))
        worse = [r for r in rows if r[2] > r[3]]
        if worse:
            out.append("- **Prediction against dead reckoning.** The tracker's median error is "
                       "above dead reckoning from the last report at horizons "
                       + ", ".join(f"{lo}–{hi} s ({t:.1f} m vs {d:.1f} m)" for lo, hi, t, d, _ in worse)
                       + ". Dead reckoning uses the aircraft's own velocity report, which the "
                       "tracker never receives (GeoPosition carries position only): the "
                       "tracker infers velocity from past positions, and none of its motion "
                       "models anticipates a turn.")
        better = [r for r in rows if r[2] <= r[3]]
        if better:
            out.append("- **Where the tracker helps.** Median error at or below dead reckoning at "
                       + ", ".join(f"{lo}–{hi} s ({t:.1f} m vs {d:.1f} m, n = {n})"
                                   for lo, hi, t, d, n in better)
                       + ". Few samples: horizons this long arise only where a real coverage "
                       "gap lengthens a scheduled one.")
        if held["no_track"]:
            out.append(f"- **No estimate.** At {held['no_track']} of {held['withheld']} withheld "
                       f"reports ({pct(held['no_track'], held['withheld'])}) the tracker offered "
                       "no confirmed or coasting track for the aircraft.")
    hs = a["cases"].get(("heldout", "all_sigma"))
    hws = a["cases"].get(("heldout", "withheld_sigma"))
    if hs and hws and held:
        def med(c, lo, hi):
            return c["horizon"].get(("tracker", lo, hi), {}).get("median")
        worse = sum(1 for lo, hi in (("2", "5"), ("20", "40"), ("40", "80"))
                    if (med(hws, lo, hi) or 0) > (med(held, lo, hi) or 0))
        verdict = ("A measured noise level alone does not fix the tracker: it predicts worse and "
                   "breaks more tracks, consistent with an estimator that then follows report "
                   "noise more closely and, with a tighter gate, loses aircraft in turns. The "
                   "50 m value, though inconsistent, "
                   "acts as smoothing the motion model needs; the mismatch is in the motion "
                   "model as much as in the measurement noise."
                   if worse >= 2 and hs["fragments"] > hall["fragments"] else
                   "A measured noise level improves prediction here.")
        out.append(f"- **With the estimated sigma ({f1(a.get('sigma_est'))} m, diagnostic).** "
                   f"NIS {f2(hs.get('nis', {}).get('mean'))}, {nis_verdict(hs.get('nis', {}))}; "
                   f"median tracker error {f1(med(hws, '2', '5'))} m at 2–5 s, "
                   f"{f1(med(hws, '20', '40'))} m at 20–40 s and {f1(med(hws, '40', '80'))} m at "
                   f"40–80 s (assumed {SIGMA_M:.0f} m: {f1(med(held, '2', '5'))}, "
                   f"{f1(med(held, '20', '40'))}, {f1(med(held, '40', '80'))} m); "
                   f"{hs['fragments']} fragmentations (assumed: {hall['fragments']}). {verdict}")
    if dflt and hall:
        members = {r.name: r for r in a["runs"]}
        lost = sorted((n for n, r in dflt.get("run", {}).items() if r["confirmed"] == "0"),
                      key=lambda n: run_speed(members[n]) or 0.0)
        if lost:
            sp = [run_speed(members[n]) for n in lost]
            ok = [run_speed(members[n]) for n, r in dflt.get("run", {}).items() if r["confirmed"] != "0"]
            out.append(f"- **Default initiation fails on fast aircraft.** With `EngineConfig` "
                       f"defaults, {len(lost)} of {len(dflt.get('run', {}))} held-out runs never "
                       f"confirm a track (median ground speed of those runs "
                       f"{f1(min(x for x in sp if x is not None) if any(sp) else None)}–"
                       f"{f1(max(x for x in sp if x is not None) if any(sp) else None)} m/s; runs "
                       f"that do confirm: {f1(min(x for x in ok if x is not None) if any(ok) else None)}–"
                       f"{f1(max(x for x in ok if x is not None) if any(ok) else None)} m/s). The "
                       "100 m/s initial velocity sigma and 3 s tentative timeout are sized for "
                       "slower simulated targets; the launcher's ADS-B settings exist for this "
                       "reason, and every ADS-B run in the app uses them.")
        out.append(f"- **Default lifecycle on real ADS-B.** With `EngineConfig` defaults the same "
                   f"input gives {dflt['tracks_started']} tracks started, "
                   f"{dflt['confirmed_tracks']} confirmed and {dflt['fragments']} fragmentations "
                   f"(ADS-B settings: {hall['tracks_started']}, {hall['confirmed_tracks']}, "
                   f"{hall['fragments']}); coverage {pct(dflt['coverage'][1], dflt['coverage'][0])} "
                   f"vs {pct(hall['coverage'][1], hall['coverage'][0])}.")
    bs = b_summary(b, "heldout") if b["rows"] else None
    if bs and "budget" in bs:
        bt = b_summary(b, "heldout", True)
        bst = b_summary(b, "heldout", False)
        if bt["real"]["scored"] and bst["real"]["scored"]:
            out.append(f"- **Manoeuvres.** With simulated sensors, the median RMSE of manoeuvring "
                       f"windows is {f1(bt['real']['rmse']['median'])} m (p95 "
                       f"{f1(bt['real']['rmse']['p95'])} m) against "
                       f"{f1(bst['real']['rmse']['median'])} m for straight ones; the "
                       f"constant-velocity controls of the same windows give "
                       f"{f1(bt['cv']['rmse']['median'])} m and {f1(bst['cv']['rmse']['median'])} m.")
        if bs["real"]["id_switches"] or bs["real"]["false_tracks"]:
            out.append(f"- **Real motion breaks simulated-radar tracks.** On real paths "
                       f"{bs['real']['id_switches']} ID switches and {bs['real']['false_tracks']} "
                       f"false tracks over {bs['real']['runs']} held-out runs, against "
                       f"{bs['cv']['id_switches']} and {bs['cv']['false_tracks']} on the "
                       "constant-velocity controls with the same sensors and seeds.")
        tb = b_summary(b, "tuning")
        if tb["real"]["rms"] and bs["real"]["rms"]:
            out.append(f"- **Large spread between flights.** The tuning windows give an RMS of "
                       f"{f1(tb['real']['rms'])} m against {f1(bs['real']['rms'])} m held out "
                       f"(controls {f1(tb['cv']['rms'])} and {f1(bs['cv']['rms'])} m): with "
                       f"{len({w.flight.name for w in b['windows'].values()})} flights "
                       "contributing windows, which flights land in a portion moves the result "
                       "by more than the sensor noise does.")
        miss = bs["real"]["missed"], bs["real"]["eligible"]
        if miss[0]:
            out.append(f"- **Losses within the match distance.** {miss[0]} of {miss[1]} covered "
                       f"target-instants ({pct(*miss)}) were not matched within "
                       f"{B_MATCH_DISTANCE_M:.0f} m on real paths (control: "
                       f"{pct(bs['cv']['missed'], bs['cv']['eligible'])}).")
    out.append("")
    return "\n".join(out)


def limitations(a: dict, flights: list[FlightInfo], synthetic: bool) -> str:
    n_air = len({fi.aircraft for fi in flights})
    multi = any(len({fi.aircraft for fi, _ in r.members}) > 1 for r in a["runs"])
    out = ["## Limitations\n"]
    if synthetic:
        out.append("- **Synthetic input.** Nothing on this page is a real-data measurement.")
    out.append(f"- **Sample.** {len(flights)} flights of {n_air} aircraft. The only real state "
               "vectors OpenSky still publishes openly are short incident extracts (diversions, "
               "emergencies, unusual flights), so the sample is small and biased towards "
               "unusual manoeuvres; it is not a traffic survey.")
    if not multi:
        out.append("- **One aircraft per run.** Each extract holds a single aircraft, so no run "
                   "has two aircraft in the same airspace: global nearest neighbour reduces to "
                   "sequential gating of one track, and identity swaps, mixed tracks and dense "
                   "terminal-area traffic cannot occur here. The metrics are implemented and "
                   "tested on synthetic multi-aircraft input (`sim/tests/test_realeval.py`); "
                   "measuring them on real traffic needs a multi-aircraft recording.")
    out.append("- **Sequential association.** Aircraft report at their own times, so a real "
               "ADS-B batch is almost always a group of one measurement: global nearest "
               "neighbour then behaves like sequential nearest-neighbour assignment, even with "
               "many aircraft.")
    out.append("- **2-D only.** The tracker discards altitude; vertical motion (climbs, descents, "
               "altitude separation of crossing aircraft) is not evaluated. Positions are "
               "compared horizontally in a tangent plane.")
    out.append("- **No independent truth.** Evaluation A scores against other ADS-B reports "
               "(which carry their own noise and OpenSky's receive-time stamping), and "
               "evaluation B's truth is interpolated ADS-B; both measure consistency with the "
               "aircraft's own reports, not absolute accuracy.")
    out.append(f"- **Isolated outliers pass cleaning.** The jump rule only rejects positions "
               f"implying more than {realclean.MAX_SPEED_MPS:.0f} m/s after "
               f"{realclean.JUMP_SLACK_M:.0f} m of slack, so a single report a few hundred metres "
               "off survives; the leave-one-out maxima show them. They enter evaluation B's truth "
               "and evaluation A's withheld targets as they are (the rule was fixed before the "
               "results and is not tightened after seeing them).")
    out.append("- **Simulated sensors in B.** Radar and camera noise there still match the "
               "filter's assumptions exactly; only the motion is real.")
    out.append("- **OpenSky timing.** `time_position` in the live API has one-second resolution "
               "and positions can repeat; the poll emulation reproduces this, the *all* feed "
               "uses the archive's millisecond `lastposupdate`.")
    out.append("")
    return "\n".join(out)


# ---- commands ----------------------------------------------------------------------

def run(args) -> int:
    manifest = load_manifest(args.manifest)
    datasets = load_datasets(manifest, args.cache)
    flights, removed, removed_total = label_flights(datasets)
    tune = [f for f in flights if f.split == "tuning"]
    held = [f for f in flights if f.split == "heldout"]
    if not held:
        raise EvalError("no held-out flight after cleaning (too few flights to split)")
    loo_tune = {m: loo_stats(tune or held, m) for m in recorded.METHODS}
    loo_held = {m: loo_stats(held, m) for m in recorded.METHODS}
    method = choose_method(loo_tune)
    sigma_est = estimate_sigma(loo_tune[method])
    for tool in ("sifted_realeval", "sifted_eval"):
        if not (args.bin_dir / tool).exists():
            raise EvalError(f"{args.bin_dir / tool} not found: build the release preset "
                            "(cmake --preset release && cmake --build --preset release) "
                            "or pass --bin-dir")
    args.work.mkdir(parents=True, exist_ok=True)
    a = evaluation_a(flights, args.work, args.bin_dir, sigma_est)
    a["sigma_est"] = sigma_est
    b = evaluation_b(flights, method, args.work, args.bin_dir)
    text = render(manifest, datasets, flights, removed, removed_total, loo_tune, loo_held, method,
                  a, b, args.command)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(text, encoding="utf-8", newline="\n")
    print(f"realeval: wrote {args.out}", file=sys.stderr)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted_sim.realeval", description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("fetch", "run"):
        p = sub.add_parser(name)
        p.add_argument("--manifest", type=pathlib.Path, default=DEFAULT_MANIFEST)
        p.add_argument("--cache", type=pathlib.Path, default=DEFAULT_CACHE)
        if name == "run":
            p.add_argument("--work", type=pathlib.Path, default=DEFAULT_WORK)
            p.add_argument("--bin-dir", type=pathlib.Path, default=DEFAULT_BIN)
            p.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    d = sub.add_parser("describe")
    d.add_argument("files", nargs="+", type=pathlib.Path)
    args = ap.parse_args(argv)
    try:
        if args.cmd == "fetch":
            fetch(load_manifest(args.manifest), args.cache)
            return 0
        if args.cmd == "describe":
            for f in args.files:
                reports, rd, _ = read_dataset(f)
                print(json.dumps({"file": f.name, "bytes": f.stat().st_size,
                                  "sha256": sha256_of(f), **describe(reports, rd.rows)}))
            return 0
        # The reproduction command printed in the report: defaults are left out so
        # the text does not depend on where the repository is checked out.
        shown = ["python3 -m sifted_sim.realeval run"]
        # --work and --bin-dir change where files go, not what they say.
        for flag, default in (("--manifest", DEFAULT_MANIFEST), ("--cache", DEFAULT_CACHE),
                              ("--out", DEFAULT_OUT)):
            v = getattr(args, flag[2:].replace("-", "_"))
            if v.resolve() != default.resolve():
                shown.append(f"{flag} {v.name if flag != '--out' else v.as_posix()}")
        args.command = " ".join(shown)
        return run(args)
    except EvalError as e:
        print(f"realeval: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
