"""Named, reproducible scenarios: sensors, a target trajectory, and a seed.

Noise levels are those of a modest surveillance radar: 10 m range and 0.2 deg
bearing (about 17 m cross-range at 5 km), scanning at 1 Hz.

    straight_crossing  Target flies outbound almost exactly along the radar's
                       +/-pi bearing seam (directly behind it) and creeps across
                       it, so for tens of seconds noisy measurements and the
                       prediction land on opposite sides of the seam. This
                       exercises bearing wrap-around end to end.
    turn               Radar offset from the origin and rotated, so the pose
                       transform matters. Target flies east, makes a 90 deg
                       right turn at 3 deg/s (about 3 m/s^2), then flies south.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .radar import RadarParams
from .trajectory import Straight, TargetState, Trajectory, Turn

SIGMA_RANGE_M = 10.0
SIGMA_BEARING_RAD = math.radians(0.2)
RATE_HZ = 1.0


@dataclass(frozen=True)
class Scenario:
    name: str
    radar: RadarParams
    trajectory: Trajectory
    start_ns: int = 0


def straight_crossing() -> Scenario:
    radar = RadarParams(sensor_id=1, x_m=0.0, y_m=0.0, yaw_rad=0.0,
                        sigma_range_m=SIGMA_RANGE_M, sigma_bearing_rad=SIGMA_BEARING_RAD,
                        rate_hz=RATE_HZ, seed=1001)
    traj = Trajectory(TargetState(-3000.0, -60.0, -60.0, 2.0), [Straight(90.0)])
    return Scenario("straight_crossing", radar, traj)


def turn() -> Scenario:
    radar = RadarParams(sensor_id=2, x_m=1000.0, y_m=-500.0, yaw_rad=0.5,
                        sigma_range_m=SIGMA_RANGE_M, sigma_bearing_rad=SIGMA_BEARING_RAD,
                        rate_hz=RATE_HZ, seed=2002)
    traj = Trajectory(TargetState(-3000.0, 2000.0, 60.0, 0.0),
                      [Straight(30.0), Turn(30.0, -math.radians(3.0)), Straight(30.0)])
    return Scenario("turn", radar, traj, start_ns=1_700_000_000 * 1_000_000_000)


SCENARIOS = {s.__name__: s for s in (straight_crossing, turn)}
