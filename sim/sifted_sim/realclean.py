"""Cleaning real ADS-B reports into flights, with every removal counted.

The rules are general (no flight, airport, region, date or address is named
anywhere) and their thresholds are module constants, recorded in the report
(docs/real-data-evaluation.md). They were fixed before any tracking result
was looked at. Applied per aircraft address, in report-time order:

  on_ground           surface reports; the tracker models airborne motion.
  duplicate_time      same position time as the previous kept report (a state
                      vector re-emitted without a new position).
  repeated_position   same latitude and longitude as the previous kept report
                      at a later time: a stale position carried forward, not a
                      new observation (decoded positions move by more than the
                      CPR resolution between airborne messages).
  jump                the implied ground speed from the previous kept report,
                      after allowing JUMP_SLACK_M of position noise, exceeds
                      MAX_SPEED_MPS: physically impossible for the aircraft
                      in these data. After JUMP_RESET consecutive jumps the
                      previous kept report is the suspect one, so a new segment
                      starts from the rejected run instead.
  gap split           a gap longer than MAX_GAP_S ends a flight; reports after
                      it start a new one (an aircraft reappearing, or the same
                      address on a later flight).
  short_segment       flights shorter than MIN_FLIGHT_S or with fewer than
                      MIN_REPORTS reports are dropped whole.
  inconsistent_identity
                      every report of an address whose jump rate exceeds
                      MAX_JUMP_FRACTION: two transmitters sharing one address
                      (reused or invalid addresses) cannot be separated by
                      kinematics alone without deciding the answer, so the
                      address is excluded.

Distances are ECEF chords (sifted_sim.geo), so a dateline crossing is not a
jump.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Iterable

from . import geo
from .statevectors import Report

MAX_SPEED_MPS = 400.0       # above any subsonic ground speed in a jet stream
JUMP_SLACK_M = 200.0        # position noise allowance (several NACp-7 radii)
JUMP_RESET = 5
MAX_GAP_S = 60.0            # ~ a coverage hole, or a different flight
MIN_FLIGHT_S = 300.0
MIN_REPORTS = 100
MAX_JUMP_FRACTION = 0.05

CHUNK_RADIUS_M = 300_000.0  # Evaluation A: reports per engine run lie within this of its origin

PARAMETERS = {
    "MAX_SPEED_MPS": MAX_SPEED_MPS, "JUMP_SLACK_M": JUMP_SLACK_M, "JUMP_RESET": JUMP_RESET,
    "MAX_GAP_S": MAX_GAP_S, "MIN_FLIGHT_S": MIN_FLIGHT_S, "MIN_REPORTS": MIN_REPORTS,
    "MAX_JUMP_FRACTION": MAX_JUMP_FRACTION, "CHUNK_RADIUS_M": CHUNK_RADIUS_M,
}


@dataclass(frozen=True)
class Flight:
    """A cleaned, time-ordered run of one aircraft's airborne reports."""

    dataset: str
    icao24: str  # identity: for scoring and grouping only, never for tracking or display
    index: int   # flight number of this address within the dataset
    reports: tuple[Report, ...]

    @property
    def start_ms(self) -> int:
        return self.reports[0].t_ms

    @property
    def end_ms(self) -> int:
        return self.reports[-1].t_ms

    @property
    def duration_s(self) -> float:
        return (self.end_ms - self.start_ms) / 1000.0


def _segments(reports: list[Report], removed: Counter) -> tuple[list[list[Report]], int]:
    """Per-address rules up to the gap split. Returns (segments, jumps)."""
    segs: list[list[Report]] = []
    cur: list[Report] = []
    rejected: list[Report] = []  # consecutive jumps since the last kept report
    jumps = 0
    for r in reports:
        if r.on_ground:
            removed["on_ground"] += 1
            continue
        if cur:
            last = cur[-1]
            dt = (r.t_ms - last.t_ms) / 1000.0
            if dt == 0.0:
                removed["duplicate_time"] += 1
                continue
            if r.lat == last.lat and r.lon == last.lon:
                removed["repeated_position"] += 1
                continue
            if dt > MAX_GAP_S:
                removed["jump"] += len(rejected)
                segs.append(cur)
                cur, rejected = [r], []
                continue
            d = geo.chord_m(last.lat, last.lon, r.lat, r.lon)
            if d - JUMP_SLACK_M > MAX_SPEED_MPS * dt:
                jumps += 1
                rejected.append(r)
                if len(rejected) >= JUMP_RESET:
                    # The kept track is the outlier: restart from the rejected run.
                    segs.append(cur)
                    cur, rejected = [rejected[-1]], []
                    removed["jump"] += JUMP_RESET - 1
                continue
            removed["jump"] += len(rejected)
            rejected = []
        cur.append(r)
    removed["jump"] += len(rejected)
    if cur:
        segs.append(cur)
    return segs, jumps


def clean(dataset: str, reports: Iterable[Report]) -> tuple[list[Flight], Counter]:
    """Flights of one dataset and the count of reports removed, by reason.

    Every input report is either in exactly one flight or counted once.
    """
    by_icao: dict[str, list[Report]] = defaultdict(list)
    for r in reports:
        by_icao[r.icao24].append(r)
    removed: Counter = Counter()
    flights: list[Flight] = []
    for icao in sorted(by_icao):
        rs = sorted(by_icao[icao], key=lambda r: (r.t_ms, r.row_time_s))  # stable for ties
        local: Counter = Counter()
        segs, jumps = _segments(rs, local)
        airborne = len(rs) - local["on_ground"]
        if airborne and jumps / airborne > MAX_JUMP_FRACTION:
            removed["inconsistent_identity"] += len(rs) - local["on_ground"]
            removed["on_ground"] += local["on_ground"]
            continue
        removed.update(local)
        index = 0
        for seg in segs:
            dur = (seg[-1].t_ms - seg[0].t_ms) / 1000.0
            if dur < MIN_FLIGHT_S or len(seg) < MIN_REPORTS:
                removed["short_segment"] += len(seg)
                continue
            flights.append(Flight(dataset, icao, index, tuple(seg)))
            index += 1
    return flights, removed


def chunks(flight: Flight, radius_m: float = CHUNK_RADIUS_M) -> list[tuple[Report, ...]]:
    """Consecutive runs of a flight's reports that lie within radius_m of the
    run's first report (its tangent-plane origin in Evaluation A)."""
    out: list[tuple[Report, ...]] = []
    cur: list[Report] = []
    for r in flight.reports:
        if cur and geo.chord_m(cur[0].lat, cur[0].lon, r.lat, r.lon) > radius_m:
            out.append(tuple(cur))
            cur = []
        cur.append(r)
    if cur:
        out.append(tuple(cur))
    return out
