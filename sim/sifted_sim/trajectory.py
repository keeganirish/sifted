"""Ground-truth target trajectories in the local tracking frame.

Frame and angle conventions match the C++ core (src/sifted/core/sensor.hpp):
x east, y north, metres; headings are radians counter-clockwise from +x.

A trajectory is an initial state plus a list of segments, each either straight
(constant velocity) or a coordinated turn (constant speed, constant turn rate).
Positions are evaluated in closed form, so the truth is exact at any time and
does not depend on a simulation step size.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class TargetState:
    x_m: float
    y_m: float
    vx_mps: float
    vy_mps: float


@dataclass(frozen=True)
class Straight:
    duration_s: float


@dataclass(frozen=True)
class Turn:
    duration_s: float
    rate_rad_s: float  # > 0 turns left (counter-clockwise), < 0 turns right


Segment = Straight | Turn


def _advance(s: TargetState, seg: Segment, t: float) -> TargetState:
    """State `t` seconds into `seg` (0 <= t <= seg.duration_s), starting from `s`."""
    if isinstance(seg, Straight) or seg.rate_rad_s == 0.0:
        return TargetState(s.x_m + s.vx_mps * t, s.y_m + s.vy_mps * t, s.vx_mps, s.vy_mps)
    speed = math.hypot(s.vx_mps, s.vy_mps)
    w = seg.rate_rad_s
    h0 = math.atan2(s.vy_mps, s.vx_mps)
    h = h0 + w * t
    radius = speed / w
    return TargetState(
        s.x_m + radius * (math.sin(h) - math.sin(h0)),
        s.y_m - radius * (math.cos(h) - math.cos(h0)),
        speed * math.cos(h),
        speed * math.sin(h),
    )


class Trajectory:
    def __init__(self, start: TargetState, segments: list[Segment]) -> None:
        if not segments:
            raise ValueError("a trajectory needs at least one segment")
        for seg in segments:
            if not seg.duration_s > 0.0:
                raise ValueError("segment durations must be positive")
        self.start = start
        self.segments = list(segments)
        # State at the start of each segment, computed once.
        self._starts = [start]
        for seg in self.segments[:-1]:
            self._starts.append(_advance(self._starts[-1], seg, seg.duration_s))

    @property
    def duration_s(self) -> float:
        return sum(seg.duration_s for seg in self.segments)

    def state_at(self, t: float) -> TargetState:
        if t < 0.0 or t > self.duration_s:
            raise ValueError(f"t={t} outside trajectory [0, {self.duration_s}]")
        for seg, s0 in zip(self.segments, self._starts):
            if t <= seg.duration_s:
                return _advance(s0, seg, t)
            t -= seg.duration_s
        return _advance(self._starts[-1], self.segments[-1], self.segments[-1].duration_s)
