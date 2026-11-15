"""Radar-like sensor: noisy range and bearing to a target, at a fixed rate.

Measurement model (identical to the C++ EKF in src/sifted/estimation/):

    range   = |target - sensor|                                  + N(0, sigma_range^2)
    bearing = wrap(atan2(dy, dx) - yaw)                          + N(0, sigma_bearing^2)

Bearings are relative to the sensor's boresight and wrapped into [-pi, pi].

Determinism: noise comes from random.Random(seed).random(), whose output for a
given seed is guaranteed stable across Python versions, turned into normals by
our own Box-Muller transform rather than random.gauss(), so the sequence does
not depend on CPython's choice of normal sampler. Timestamps are integers.

Multi-target scans (observe_targets, M2) add coverage (maximum range), a
detection probability, uniform clutter (false alarms) and dropout windows.
Detection draws, clutter and dropouts use a second seeded stream, separate
from the measurement-noise stream, so turning failure modes on or off does
not change the noise of the detections that remain. Within a scan,
detections are reported in bearing order, as a rotating radar would, never in
target order.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from . import wire
from .sensors import SensorConfig
from .trajectory import Target, TargetState, Trajectory

NS_PER_S = 1_000_000_000


def wrap_angle(a: float) -> float:
    """Into [-pi, pi]; math.remainder is exact, like std::remainder in C++."""
    return math.remainder(a, 2.0 * math.pi)


class Normal:
    """Standard normal samples from a seeded Mersenne Twister via Box-Muller."""

    def __init__(self, seed: int) -> None:
        self._rng = random.Random(seed)
        self._spare: float | None = None

    def sample(self) -> float:
        if self._spare is not None:
            z, self._spare = self._spare, None
            return z
        u1 = 1.0 - self._rng.random()  # (0, 1], so log() is finite
        u2 = self._rng.random()
        mag = math.sqrt(-2.0 * math.log(u1))
        self._spare = mag * math.sin(2.0 * math.pi * u2)
        return mag * math.cos(2.0 * math.pi * u2)


@dataclass(frozen=True)
class RadarParams:
    sensor_id: int
    x_m: float
    y_m: float
    yaw_rad: float
    sigma_range_m: float
    sigma_bearing_rad: float
    rate_hz: float
    seed: int
    # Failure modes and coverage (used by observe_targets; the M1 single-target
    # observe() ignores them).
    max_range_m: float = math.inf
    p_detect: float = 1.0
    clutter_per_scan: float = 0.0  # mean false alarms per scan, uniform over the coverage disc
    dropouts: tuple[tuple[float, float], ...] = ()  # [start_s, end_s) windows with no output

    def __post_init__(self) -> None:
        if not (self.sigma_range_m > 0.0 and self.sigma_bearing_rad > 0.0):
            raise ValueError("noise sigmas must be positive")
        if not self.rate_hz > 0.0:
            raise ValueError("rate must be positive")
        if not (0.0 <= self.p_detect <= 1.0 and self.max_range_m > 0.0):
            raise ValueError("p_detect must be in [0, 1] and max_range_m positive")
        if self.clutter_per_scan < 0.0 or (self.clutter_per_scan > 0.0 and
                                            not math.isfinite(self.max_range_m)):
            raise ValueError("clutter needs a finite max_range_m")

    @property
    def config(self) -> SensorConfig:
        return SensorConfig(self.sensor_id, wire.RANGE_BEARING, self.x_m, self.y_m, self.yaw_rad)

    @property
    def period_ns(self) -> int:
        return round(NS_PER_S / self.rate_hz)


@dataclass(frozen=True)
class Sample:
    """One emitted measurement and the true target state at its timestamp."""

    measurement: wire.Measurement
    truth: TargetState


@dataclass(frozen=True)
class Detection:
    """One emitted measurement and where it came from (None: clutter)."""

    measurement: wire.Measurement
    target_id: int | None


def in_dropout(dropouts: tuple[tuple[float, float], ...], t_s: float) -> bool:
    return any(a <= t_s < b for a, b in dropouts)


def poisson(rng: random.Random, mean: float) -> int:
    """Knuth's method on rng.random(): deterministic for a given stream."""
    if mean <= 0.0:
        return 0
    limit, k, p = math.exp(-mean), 0, 1.0
    while True:
        p *= rng.random()
        if p <= limit:
            return k
        k += 1


def true_range_bearing(p: RadarParams, s: TargetState) -> tuple[float, float]:
    dx, dy = s.x_m - p.x_m, s.y_m - p.y_m
    return math.hypot(dx, dy), wrap_angle(math.atan2(dy, dx) - p.yaw_rad)


def observe(p: RadarParams, trajectory: Trajectory, start_ns: int = 0) -> list[Sample]:
    """Scans the target every period from t=0 to the end of the trajectory.

    `start_ns` is the timestamp of t=0. Each sample's truth is the trajectory
    state at exactly the measurement's (integer) timestamp.
    """
    noise = Normal(p.seed)
    samples: list[Sample] = []
    seq = 0
    end_ns = int(trajectory.duration_s * NS_PER_S)
    for offset_ns in range(0, end_ns + 1, p.period_ns):
        truth = trajectory.state_at(offset_ns / NS_PER_S)
        r, b = true_range_bearing(p, truth)
        r = max(0.0, r + p.sigma_range_m * noise.sample())
        b = wrap_angle(b + p.sigma_bearing_rad * noise.sample())
        m = wire.Measurement(
            sensor_id=p.sensor_id, kind=wire.RANGE_BEARING, seq=seq,
            time_ns=start_ns + offset_ns, z=(r, b, 0.0),
            sigma=(p.sigma_range_m, p.sigma_bearing_rad, 0.0),
        )
        samples.append(Sample(m, truth))
        seq += 1
    return samples


def observe_targets(p: RadarParams, targets: list[Target], duration_s: float,
                    start_ns: int = 0) -> list[Detection]:
    """Scans every period from t=0 to duration_s; returns detections in send order.

    A target is detectable when it exists at the scan time and is within
    max_range_m; it is then detected with probability p_detect. Clutter points
    are uniform over the coverage disc. Nothing is emitted inside a dropout.
    """
    noise = Normal(p.seed)
    events = random.Random(p.seed ^ 0x5EED_F00D)
    out: list[Detection] = []
    seq = 0
    end_ns = int(duration_s * NS_PER_S)
    for offset_ns in range(0, end_ns + 1, p.period_ns):
        t_s = offset_ns / NS_PER_S
        if in_dropout(p.dropouts, t_s):
            continue
        scan: list[tuple[float, float, int | None]] = []
        for tgt in sorted(targets, key=lambda x: x.target_id):
            state = tgt.state_at(t_s)
            if state is None:
                continue
            r_true, b_true = true_range_bearing(p, state)
            # Noise and the detection draw are taken for every existing target,
            # detected or not, so each stream advances independently.
            r = max(0.0, r_true + p.sigma_range_m * noise.sample())
            b = wrap_angle(b_true + p.sigma_bearing_rad * noise.sample())
            detected = events.random() < p.p_detect
            if r_true <= p.max_range_m and detected:
                scan.append((b, r, tgt.target_id))
        for _ in range(poisson(events, p.clutter_per_scan)):
            r = p.max_range_m * math.sqrt(events.random())  # uniform over the disc
            b = wrap_angle(2.0 * math.pi * events.random())
            scan.append((b, r, None))
        scan.sort(key=lambda d: (d[0], d[1]))  # bearing order, like a rotating antenna
        for b, r, tid in scan:
            m = wire.Measurement(
                sensor_id=p.sensor_id, kind=wire.RANGE_BEARING, seq=seq & 0xFFFFFFFF,
                time_ns=start_ns + offset_ns, z=(r, b, 0.0),
                sigma=(p.sigma_range_m, p.sigma_bearing_rad, 0.0),
            )
            out.append(Detection(m, tid))
            seq += 1
    return out
