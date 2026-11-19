"""Recorded truth trajectories: a real flight path as a `Target` trajectory.

A RecordedTrajectory has the same interface as trajectory.Trajectory
(`duration_s`, `state_at(t)`), so it plugs into trajectory.Target and is
observed by the existing radar and camera simulators unchanged. The
closed-form trajectories are not touched.

Knots are position reports (time, x, y) in a local frame, each with a
velocity (vx, vy) where one was reported. Between knots the path is
interpolated by one of:

  hermite  cubic Hermite on each interval, matching position and velocity at
           both ends. It reproduces constant-velocity and constant-
           acceleration motion exactly, and a coordinated turn to third order,
           using the aircraft's own velocity reports (OpenSky velocity and
           true track) rather than guessing slopes from neighbouring noisy
           positions. A knot without a reported velocity gets the
           finite-difference slope of its neighbours.
  linear   straight lines between knots, constant velocity per interval.

Which one evaluations use is decided on data, by leave-one-out on the tuning
flights (sifted_sim.realeval): each interior knot is removed, predicted from
its two neighbours, and the prediction compared with the report. That error
mixes interpolation error over twice the real spacing with the report's own
noise, so it bounds the interpolation error at the real spacing from above.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from .trajectory import TargetState

METHODS = ("hermite", "linear")


@dataclass(frozen=True)
class Knot:
    t_s: float
    x: float
    y: float
    vx: float | None = None
    vy: float | None = None


def _slopes(knots: list[Knot]) -> list[tuple[float, float]]:
    out = []
    n = len(knots)
    for i, k in enumerate(knots):
        if k.vx is not None and k.vy is not None:
            out.append((k.vx, k.vy))
            continue
        a, b = knots[max(i - 1, 0)], knots[min(i + 1, n - 1)]
        dt = b.t_s - a.t_s
        out.append(((b.x - a.x) / dt, (b.y - a.y) / dt) if dt > 0 else (0.0, 0.0))
    return out


def _hermite(t: float, k0: Knot, k1: Knot, v0, v1) -> TargetState:
    h = k1.t_s - k0.t_s
    s = (t - k0.t_s) / h
    s2, s3 = s * s, s * s * s
    h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + s, -2 * s3 + 3 * s2, s3 - s2
    d00, d10, d01, d11 = 6 * s2 - 6 * s, 3 * s2 - 4 * s + 1, -6 * s2 + 6 * s, 3 * s2 - 2 * s
    x = h00 * k0.x + h10 * h * v0[0] + h01 * k1.x + h11 * h * v1[0]
    y = h00 * k0.y + h10 * h * v0[1] + h01 * k1.y + h11 * h * v1[1]
    vx = (d00 * k0.x + d01 * k1.x) / h + d10 * v0[0] + d11 * v1[0]
    vy = (d00 * k0.y + d01 * k1.y) / h + d10 * v0[1] + d11 * v1[1]
    return TargetState(x, y, vx, vy)


def _linear(t: float, k0: Knot, k1: Knot) -> TargetState:
    h = k1.t_s - k0.t_s
    s = (t - k0.t_s) / h
    vx, vy = (k1.x - k0.x) / h, (k1.y - k0.y) / h
    return TargetState(k0.x + s * (k1.x - k0.x), k0.y + s * (k1.y - k0.y), vx, vy)


def interpolate(t: float, k0: Knot, k1: Knot, v0, v1, method: str) -> TargetState:
    if method == "hermite":
        return _hermite(t, k0, k1, v0, v1)
    if method == "linear":
        return _linear(t, k0, k1)
    raise ValueError(f"unknown interpolation method {method!r}")


class RecordedTrajectory:
    """A path through time-ordered knots, starting at t = 0 (the first knot)."""

    def __init__(self, knots: list[Knot], method: str = "hermite") -> None:
        if len(knots) < 2:
            raise ValueError("a recorded trajectory needs at least two knots")
        if method not in METHODS:
            raise ValueError(f"unknown interpolation method {method!r}")
        if knots[0].t_s != 0.0:
            raise ValueError("the first knot must be at t = 0")
        for a, b in zip(knots, knots[1:]):
            if not b.t_s > a.t_s:
                raise ValueError("knot times must be strictly increasing")
        for k in knots:
            vals = [k.t_s, k.x, k.y] + [v for v in (k.vx, k.vy) if v is not None]
            if not all(math.isfinite(v) for v in vals):
                raise ValueError("knots must be finite")
        self.knots = list(knots)
        self.method = method
        self._t = [k.t_s for k in knots]
        self._v = _slopes(self.knots)

    @property
    def duration_s(self) -> float:
        return self._t[-1]

    def state_at(self, t: float) -> TargetState:
        if t < 0.0 or t > self.duration_s:
            raise ValueError(f"t={t} outside trajectory [0, {self.duration_s}]")
        i = min(max(bisect.bisect_right(self._t, t) - 1, 0), len(self._t) - 2)
        return interpolate(t, self.knots[i], self.knots[i + 1], self._v[i], self._v[i + 1],
                           self.method)


def leave_one_out(knots: list[Knot], method: str) -> list[tuple[float, float]]:
    """(neighbour spacing [s], position error [m]) for every interior knot,
    predicted from its two neighbours alone."""
    v = _slopes(knots)
    out = []
    for i in range(1, len(knots) - 1):
        a, k, b = knots[i - 1], knots[i], knots[i + 1]
        p = interpolate(k.t_s, a, b, v[i - 1], v[i + 1], method)
        out.append((b.t_s - a.t_s, math.hypot(p.x_m - k.x, p.y_m - k.y)))
    return out
