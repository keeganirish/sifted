"""Ground truth for the operator view's overlay: parsing, time alignment, matching.

This is a display aid. The authoritative accuracy figures are the scorer in
eval/sifted/eval/scenario.cpp and the scorecard it writes (docs/evaluation.md).
Nothing here feeds the daemon, the engine, the recordings or a replay.

The launcher serves these functions to the browser, so the logic exists once
and is unit-tested here (tests/test_truth.py); app.js only draws the result.

Clocks. A snapshot's time_ns is the engine's clock, which is the measurement
timestamps, and the simulator stamps measurements and truth from the same
scenario start_ns. Playback speed, network delay and replay change when a
snapshot arrives, never the time it carries, so truth is simply evaluated at
time_ns with no offset. A snapshot outside the truth's time span (a sender
using --rebase-now, say) is reported as such rather than matched against
nothing.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field

from .generate import TRUTH_HEADER_V1, TRUTH_HEADER_V2

# Mirrors kMatchDistanceM and kMissBurnInS in eval/sifted/eval/scenario.hpp, the
# scoring rule this display follows; tests/test_truth.py fails if they diverge.
MATCH_DISTANCE_M = 100.0
MISS_BURN_IN_S = 5.0
# The scorer's "reported" tracks. Tentative tracks are neither matched nor flagged.
REPORTED_STATUSES = frozenset({"confirmed", "coasting"})

TRAIL_S = 30.0  # length of the truth trail drawn behind each target
NS_PER_S = 1_000_000_000
MAX_TRUTH_BYTES = 64 * 1024 * 1024  # far above any scenario; a bound on what is parsed

# WGS-84, as in src/sifted/estimation/geodetic.cpp.
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)


class TruthError(ValueError):
    """A truth file or a comparison request is malformed."""


# ---------------------------------------------------------------------- parsing

@dataclass
class TargetTruth:
    """One target's samples, strictly increasing in time."""

    target_id: int
    times_ns: list[int] = field(default_factory=list)
    xs: list[float] = field(default_factory=list)
    ys: list[float] = field(default_factory=list)
    covered: list[bool] = field(default_factory=list)

    def first_covered_ns(self) -> int | None:
        return next((t for t, c in zip(self.times_ns, self.covered) if c), None)

    def at(self, time_ns: int) -> tuple[float, float, bool] | None:
        """(x, y, covered) interpolated linearly to time_ns, or None outside the
        sampled interval: a target is never extrapolated before it appears or
        after it leaves. Coverage is that of the latest sample at or before time_ns."""
        times = self.times_ns
        if not times or time_ns < times[0] or time_ns > times[-1]:
            return None
        i = bisect.bisect_right(times, time_ns) - 1
        if times[i] == time_ns:
            return self.xs[i], self.ys[i], self.covered[i]
        f = (time_ns - times[i]) / (times[i + 1] - times[i])
        return (self.xs[i] + f * (self.xs[i + 1] - self.xs[i]),
                self.ys[i] + f * (self.ys[i + 1] - self.ys[i]),
                self.covered[i])

    def samples_between(self, start_ns: int, end_ns: int) -> range:
        """Indices of the samples with start_ns <= t <= end_ns."""
        return range(bisect.bisect_left(self.times_ns, start_ns),
                     bisect.bisect_right(self.times_ns, end_ns))


@dataclass
class Truth:
    targets: dict[int, TargetTruth]
    start_ns: int
    end_ns: int

    def payload(self) -> dict:
        """The compact JSON form: per-target columns, times relative to start_ns."""
        return {
            "available": True,
            "match_distance_m": MATCH_DISTANCE_M,
            "start_ns": self.start_ns,
            "end_ns": self.end_ns,
            "targets": [{
                "id": t.target_id,
                "t_s": [round((n - self.start_ns) / NS_PER_S, 9) for n in t.times_ns],
                "x": [round(v, 3) for v in t.xs],
                "y": [round(v, 3) for v in t.ys],
                "covered": [int(c) for c in t.covered],
            } for _, t in sorted(self.targets.items())],
        }


def _finite(text: str) -> float:
    v = float(text)
    if not math.isfinite(v):
        raise ValueError(f"non-finite value {text!r}")
    return v


def parse_truth(text: str) -> Truth:
    """Parses truth.csv in either layout written by sifted_sim.generate."""
    lines = text.splitlines()
    header = lines[0].strip() if lines else ""
    if header not in (TRUTH_HEADER_V1, TRUTH_HEADER_V2):
        raise TruthError(f"unrecognised truth header {header!r}")
    multi = header == TRUTH_HEADER_V2
    targets: dict[int, TargetTruth] = {}
    for number, line in enumerate(lines[1:], start=2):
        if not line.strip():
            continue
        cols = line.strip().split(",")
        if len(cols) != (7 if multi else 5):
            raise TruthError(f"truth line {number}: wrong number of fields")
        try:
            time_ns, x, y = int(cols[0]), _finite(cols[1]), _finite(cols[2])
            _finite(cols[3]), _finite(cols[4])  # velocities: validated, not displayed
            target_id = int(cols[5]) if multi else 1
            covered = int(cols[6]) if multi else 1
        except ValueError as e:
            raise TruthError(f"truth line {number}: {e}") from None
        if covered not in (0, 1):
            raise TruthError(f"truth line {number}: covered must be 0 or 1")
        t = targets.setdefault(target_id, TargetTruth(target_id))
        if t.times_ns and time_ns <= t.times_ns[-1]:
            raise TruthError(f"truth line {number}: time does not increase for target {target_id}")
        t.times_ns.append(time_ns)
        t.xs.append(x)
        t.ys.append(y)
        t.covered.append(bool(covered))
    if not targets:
        raise TruthError("truth file has no samples")
    return Truth(targets, min(t.times_ns[0] for t in targets.values()),
                 max(t.times_ns[-1] for t in targets.values()))


# --------------------------------------------------------------------- matching

def solve_assignment(cost: list[list[float | None]], unassigned_cost: float) -> list[int | None]:
    """Minimum-total-cost assignment of rows to columns (None: forbidden pair).

    The same formulation as association::solve_assignment in
    src/sifted/association/assignment.cpp: the Hungarian algorithm with one
    "stay unassigned" dummy column per row. Returns each row's column or None.
    """
    rows = len(cost)
    if rows == 0:
        return []
    cols = len(cost[0])
    allowed = [c for row in cost for c in row if c is not None]
    big = (max([unassigned_cost, *allowed]) + 1.0) * (rows + 1.0) * 2.0
    m = cols + rows

    def at(r: int, c: int) -> float:
        if c >= cols:
            return unassigned_cost
        return big if cost[r][c] is None else cost[r][c]

    u = [0.0] * (rows + 1)
    v = [0.0] * (m + 1)
    p = [0] * (m + 1)    # p[j]: row matched to column j (1-based; 0 is a sentinel)
    way = [0] * (m + 1)  # previous column on the augmenting path
    for i in range(1, rows + 1):
        p[0] = i
        j0 = 0
        minv = [math.inf] * (m + 1)
        used = [False] * (m + 1)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta, j1 = math.inf, 0
            for j in range(1, m + 1):
                if used[j]:
                    continue
                cur = at(i0 - 1, j - 1) - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j], way[j] = cur, j0
                if minv[j] < delta:
                    delta, j1 = minv[j], j
            for j in range(m + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while j0 != 0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1

    result: list[int | None] = [None] * rows
    for j in range(1, cols + 1):
        if p[j] != 0 and cost[p[j] - 1][j - 1] is not None:
            result[p[j] - 1] = j - 1
    return result


def match_targets(targets: dict[int, tuple[float, float]], tracks: dict[int, tuple[float, float]],
                  previous: dict[int, int],
                  max_distance_m: float = MATCH_DISTANCE_M) -> dict[int, int]:
    """One-to-one target -> track matches, following the scoring rule.

    A target keeps its previous track while that track is still reported, free
    and within max_distance_m, so two targets passing through the same point do
    not swap. The rest are matched by minimum total distance, with pairs further
    apart than max_distance_m forbidden.
    """
    def distance(tgt: int, trk: int) -> float:
        return math.hypot(tracks[trk][0] - targets[tgt][0], tracks[trk][1] - targets[tgt][1])

    matches: dict[int, int] = {}
    taken: set[int] = set()
    for tgt in sorted(targets):
        trk = previous.get(tgt)
        if trk in tracks and trk not in taken and distance(tgt, trk) <= max_distance_m:
            matches[tgt] = trk
            taken.add(trk)
    free_targets = [t for t in sorted(targets) if t not in matches]
    free_tracks = [t for t in sorted(tracks) if t not in taken]
    if free_targets and free_tracks:
        cost: list[list[float | None]] = []
        for tgt in free_targets:
            ds = [distance(tgt, trk) for trk in free_tracks]
            cost.append([d if d <= max_distance_m else None for d in ds])
        for tgt, col in zip(free_targets, solve_assignment(cost, max_distance_m)):
            if col is not None:
                matches[tgt] = free_tracks[col]
    return matches


# ------------------------------------------------------------------- projection

def enu_to_geodetic(origin: tuple[float, float, float], east_m: float,
                    north_m: float) -> tuple[float, float]:
    """(lat, lon) in degrees of a local point: the daemon's estimation::enu_to_geodetic
    (src/sifted/estimation/geodetic.cpp), so truth lands where a track at the same
    local position is drawn. The web view's offset() is too coarse for this: at
    latitude 51 deg its spherical Earth misplaces a point 8 km east by about 16 m."""
    lat0, lon0 = math.radians(origin[0]), math.radians(origin[1])
    alt0 = origin[2]
    sl, cl, so, co = math.sin(lat0), math.cos(lat0), math.sin(lon0), math.cos(lon0)
    n0 = _A / math.sqrt(1.0 - _E2 * sl * sl)
    x = (n0 + alt0) * cl * co - so * east_m - sl * co * north_m
    y = (n0 + alt0) * cl * so + co * east_m - sl * so * north_m
    z = (n0 * (1.0 - _E2) + alt0) * sl + cl * north_m
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _E2))
    for _ in range(6):  # the same fixed-count iteration as the daemon
        s = math.sin(lat)
        n = _A / math.sqrt(1.0 - _E2 * s * s)
        c = math.cos(lat)
        h = p / c - n if abs(c) > 1e-12 else abs(z) - n * (1.0 - _E2)
        lat = math.atan2(z, p * (1.0 - _E2 * n / (n + h)))
    return math.degrees(lat), math.degrees(math.atan2(y, x))


# ------------------------------------------------------------------- comparison

@dataclass(frozen=True)
class CompareRequest:
    time_ns: int
    origin: tuple[float, float, float]
    tracks: dict[int, tuple[float, float]]  # reported tracks only
    pairs: dict[int, int]


def _integer(v: object, what: str) -> int:
    if not isinstance(v, int) or isinstance(v, bool):
        raise TruthError(f"{what} must be an integer")
    return v


def _number(v: object, what: str) -> float:
    if not isinstance(v, (int, float)) or isinstance(v, bool) or not math.isfinite(v):
        raise TruthError(f"{what} must be a finite number")
    return float(v)


def parse_compare_request(body: dict) -> CompareRequest:
    """Validates the JSON body of a comparison, strictly: it comes over HTTP."""
    origin = body.get("origin")
    if not isinstance(origin, dict):
        raise TruthError("origin must be an object")
    lat, lon = _number(origin.get("lat"), "origin.lat"), _number(origin.get("lon"), "origin.lon")
    alt = _number(origin.get("alt", 0.0), "origin.alt")
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise TruthError("origin is out of range")
    raw_tracks = body.get("tracks")
    if not isinstance(raw_tracks, list):
        raise TruthError("tracks must be a list")
    tracks: dict[int, tuple[float, float]] = {}
    for t in raw_tracks:
        if not isinstance(t, dict) or not isinstance(t.get("status"), str):
            raise TruthError("each track needs an id, x, y and status")
        track_id = _integer(t.get("id"), "track id")
        if t["status"] in REPORTED_STATUSES:
            tracks[track_id] = (_number(t.get("x"), "track x"), _number(t.get("y"), "track y"))
    raw_pairs = body.get("pairs", {})
    if not isinstance(raw_pairs, dict):
        raise TruthError("pairs must be an object")
    pairs: dict[int, int] = {}
    for k, v in raw_pairs.items():
        try:
            pairs[int(k)] = _integer(v, "paired track id")
        except ValueError:
            raise TruthError("pairs must map target ids to track ids") from None
    return CompareRequest(_integer(body.get("time_ns"), "time_ns"), (lat, lon, alt), tracks, pairs)


def compare(truth: Truth, req: CompareRequest) -> dict:
    """Truth at the snapshot's time against its reported tracks.

    Stateless: the caller passes back the `pairs` of the previous answer, so
    each open window keeps its own matching history.

    Target states: matched; missed (covered and past the scorer's burn-in, yet
    no track within the match distance); acquiring (covered, still inside the
    burn-in); uncovered (outside radar coverage, so not expected to be tracked).
    """
    now = req.time_ns
    if not truth.start_ns <= now <= truth.end_ns:
        return {"available": True, "in_span": False, "time_ns": now, "targets": [], "tracks": {},
                "unmatched_tracks": [], "mean_error_m": None, "matched": 0, "missed": 0,
                "present": 0, "pairs": {str(k): v for k, v in req.pairs.items()}}

    present: dict[int, tuple[float, float, bool]] = {}
    for tid, target in truth.targets.items():
        state = target.at(now)
        if state is not None:
            present[tid] = state
    matches = match_targets({t: (s[0], s[1]) for t, s in present.items()}, req.tracks, req.pairs)

    targets, track_info, errors, missed = [], {}, [], 0
    trail_from = now - round(TRAIL_S * NS_PER_S)
    for tid in sorted(present):
        x, y, covered = present[tid]
        target = truth.targets[tid]
        track_id = matches.get(tid)
        error_m = None
        if track_id is not None:
            tx, ty = req.tracks[track_id]
            error_m = math.hypot(tx - x, ty - y)
            errors.append(error_m)
            track_info[track_id] = {"target": tid, "error_m": round(error_m, 3)}
            state = "matched"
        elif not covered:
            state = "uncovered"
        else:
            first = target.first_covered_ns()
            old_enough = first is not None and now - first >= MISS_BURN_IN_S * NS_PER_S
            state = "missed" if old_enough else "acquiring"
            missed += state == "missed"
        lat, lon = enu_to_geodetic(req.origin, x, y)
        trail = [enu_to_geodetic(req.origin, target.xs[i], target.ys[i])
                 for i in target.samples_between(trail_from, now)]
        trail.append((lat, lon))
        targets.append({
            "id": tid, "x": round(x, 3), "y": round(y, 3), "lat": round(lat, 8),
            "lon": round(lon, 8), "state": state, "track": track_id,
            "error_m": None if error_m is None else round(error_m, 3),
            "trail": [[round(a, 7), round(b, 7)] for a, b in trail],
        })
    pairs = {**req.pairs, **matches}  # an unmatched target remembers its last track
    return {
        "available": True, "in_span": True, "time_ns": now, "targets": targets,
        "tracks": {str(k): v for k, v in track_info.items()},
        "unmatched_tracks": sorted(t for t in req.tracks if t not in track_info),
        "mean_error_m": round(sum(errors) / len(errors), 3) if errors else None,
        "matched": len(errors), "missed": missed, "present": len(present),
        "pairs": {str(k): v for k, v in pairs.items()},
    }
