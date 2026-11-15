"""Camera-like sensor: noisy bearing to each target in its field of view.

Measurement model (identical to the C++ EKF in src/sifted/estimation/bearing_only.hpp):

    bearing = wrap(atan2(dy, dx) - yaw) + N(0, sigma_bearing^2)

A camera measures direction only; range is unobservable, so the tracker uses
its bearings to refine tracks that a position-observing sensor started.

Coverage: a target is visible when it exists, lies within max_range_m and
within +/- fov_rad/2 of the boresight. It is then detected with probability
p_detect. `phase_s` offsets the frame times from t=0 so camera and radar
measurements interleave rather than coincide. Dropout windows emit nothing.

Determinism follows radar.py: noise from Normal(seed); detection draws from a
second stream seeded from `seed`; integer nanosecond timestamps. Within one
frame, bearings are reported in increasing order (image column order).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from . import wire
from .radar import NS_PER_S, Detection, Normal, in_dropout, wrap_angle
from .sensors import SensorConfig
from .trajectory import Target


@dataclass(frozen=True)
class CameraParams:
    sensor_id: int
    x_m: float
    y_m: float
    yaw_rad: float
    sigma_bearing_rad: float
    rate_hz: float
    seed: int
    fov_rad: float = 2.0 * math.pi
    max_range_m: float = math.inf
    p_detect: float = 1.0
    phase_s: float = 0.0
    dropouts: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        if not self.sigma_bearing_rad > 0.0:
            raise ValueError("noise sigma must be positive")
        if not self.rate_hz > 0.0:
            raise ValueError("rate must be positive")
        if not (0.0 < self.fov_rad <= 2.0 * math.pi and 0.0 <= self.p_detect <= 1.0):
            raise ValueError("fov must be in (0, 2 pi] and p_detect in [0, 1]")
        if not 0.0 <= self.phase_s < 1.0 / self.rate_hz:
            raise ValueError("phase must be within one frame period")

    @property
    def config(self) -> SensorConfig:
        return SensorConfig(self.sensor_id, wire.BEARING_ONLY, self.x_m, self.y_m, self.yaw_rad)

    @property
    def period_ns(self) -> int:
        return round(NS_PER_S / self.rate_hz)


def true_bearing(p: CameraParams, x_m: float, y_m: float) -> tuple[float, float]:
    """(range, boresight-relative bearing) from the camera to (x, y)."""
    dx, dy = x_m - p.x_m, y_m - p.y_m
    return math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - p.yaw_rad)


def observe_targets(p: CameraParams, targets: list[Target], duration_s: float,
                    start_ns: int = 0) -> list[Detection]:
    noise = Normal(p.seed)
    events = random.Random(p.seed ^ 0x5EED_F00D)
    out: list[Detection] = []
    seq = 0
    phase_ns = round(p.phase_s * NS_PER_S)
    end_ns = int(duration_s * NS_PER_S)
    for offset_ns in range(phase_ns, end_ns + 1, p.period_ns):
        t_s = offset_ns / NS_PER_S
        if in_dropout(p.dropouts, t_s):
            continue
        frame: list[tuple[float, int]] = []
        for tgt in sorted(targets, key=lambda x: x.target_id):
            state = tgt.state_at(t_s)
            if state is None:
                continue
            r, b = true_bearing(p, state.x_m, state.y_m)
            # Drawn for every existing target so the streams advance independently.
            measured = wrap_angle(b + p.sigma_bearing_rad * noise.sample())
            detected = events.random() < p.p_detect
            if r <= p.max_range_m and abs(b) <= 0.5 * p.fov_rad and detected:
                frame.append((measured, tgt.target_id))
        frame.sort()
        for b, tid in frame:
            m = wire.Measurement(
                sensor_id=p.sensor_id, kind=wire.BEARING_ONLY, seq=seq & 0xFFFFFFFF,
                time_ns=start_ns + offset_ns, z=(b, 0.0, 0.0),
                sigma=(p.sigma_bearing_rad, 0.0, 0.0),
            )
            out.append(Detection(m, tid))
            seq += 1
    return out
