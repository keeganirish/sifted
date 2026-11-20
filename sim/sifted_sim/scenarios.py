"""Named, reproducible scenarios: sensors, target trajectories, and seeds.

Single-target radar scenarios (M1). Noise levels are those of a modest
surveillance radar: 10 m range and 0.2 deg bearing (about 17 m cross-range at
5 km), scanning at 1 Hz.

    straight_crossing  Target flies outbound almost exactly along the radar's
                       +/-pi bearing seam (directly behind it) and creeps across
                       it, so for tens of seconds noisy measurements and the
                       prediction land on opposite sides of the seam. This
                       exercises bearing wrap-around end to end.
    turn               Radar offset from the origin and rotated, so the pose
                       transform matters. Target flies east, makes a 90 deg
                       right turn at 3 deg/s (about 3 m/s^2), then flies south.

Multi-target, multi-sensor scenarios (M2). One radar as above but with finite
coverage (8 km), detection probability 0.9 and on average 0.5 clutter points
per scan; one camera (0.1 deg, 2 Hz, offset by 0.25 s from the radar scans so
the two interleave) at a different site, so radar range and camera bearing
constrain the target from different directions.

    crossing_pair      Two targets on straight paths crossing at a shallow
                       30 deg angle, ~4 km from the radar; for ~10 s they are
                       inside each other's gates, so association decides who
                       is who. The camera here is omnidirectional and both
                       targets cross its +/-pi bearing seam near the crossing.
    close_pass         Two targets on reciprocal headings in lanes 120 m apart
                       (about 5 radar cross-range sigmas at 5 km) pass each
                       other, plus a third, manoeuvring target elsewhere.
    formation_split    Three targets fly in close line-abreast formation
                       (lanes 45 m apart, ~3 radar range sigmas, so every
                       scan's detections fall inside several tracks' gates),
                       then split: the outer two turn away at 2 deg/s and
                       fly off on diverging headings. Built to separate global
                       nearest neighbour from greedy assignment (M3): a greedy
                       pass takes the single closest pair first and regularly
                       hands a track its neighbour's detection, which shows up
                       as ID switches and broken tracks after the split.
    demo               For the demo runbook (docs/demo.md), 150 s: a pair
                       crossing at a shallow angle near the middle of
                       coverage, one target flying in from outside coverage
                       and one flying out of it, one turning, all seen by
                       radar and camera. A network outage is added live by
                       the impairment proxy (sifted_sim.impair --outage).
    enter_leave        Targets enter coverage, leave it, appear mid-run and
                       vanish mid-run (e.g. landing), with lower detection
                       probability (0.85), more clutter and a 6 s camera
                       dropout. Exercises tentative -> confirmed -> coasting ->
                       dropped.

Parametric manoeuvre family (motion-model evaluation, ADR 0014). Not
committed as fixtures: `sifted_sim.manoeuvres` writes Monte Carlo sets of them
into the build directory.

    manoeuvre(load_g, speed_mps, seed)
                       One radar at the origin (10 m, 0.2 deg, 1 Hz, Pd 0.9,
                       0.5 clutter points per scan, 60 km coverage), one target
                       starting at (-6000, 15000) m flying east: 30 s straight,
                       a constant-rate turn of load_g g lasting
                       min(180 deg / rate, 40 s), then 30 s straight. Right
                       turn by default. The radar seed is the noise seed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .camera import CameraParams
from .radar import RadarParams
from .trajectory import Straight, Target, TargetState, Trajectory, Turn

SIGMA_RANGE_M = 10.0
SIGMA_BEARING_RAD = math.radians(0.2)
RATE_HZ = 1.0


@dataclass(frozen=True)
class Scenario:
    name: str
    radar: RadarParams
    trajectory: Trajectory
    start_ns: int = 0


@dataclass(frozen=True)
class MultiScenario:
    """Several targets observed by several sensors.

    Truth is reported every 1/truth_rate_hz seconds for every target that
    exists, flagged `covered` when it lies inside the coverage of at least one
    radar (the sensors that can start and hold a track), i.e. when the
    tracker is expected to hold it.
    """

    name: str
    radars: list[RadarParams]
    cameras: list[CameraParams]
    targets: list[Target]
    duration_s: float
    start_ns: int = 0
    truth_rate_hz: float = 1.0
    notes: list[str] = field(default_factory=list)


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


def _radar(sensor_id: int, seed: int, **kw) -> RadarParams:
    base = dict(x_m=0.0, y_m=0.0, yaw_rad=0.0, sigma_range_m=SIGMA_RANGE_M,
                sigma_bearing_rad=SIGMA_BEARING_RAD, rate_hz=RATE_HZ, max_range_m=8000.0,
                p_detect=0.9, clutter_per_scan=0.5)
    base.update(kw)
    return RadarParams(sensor_id=sensor_id, seed=seed, **base)


def _camera(sensor_id: int, seed: int, **kw) -> CameraParams:
    base = dict(x_m=-3000.0, y_m=3000.0, yaw_rad=-math.pi / 4,
                sigma_bearing_rad=math.radians(0.1), rate_hz=2.0, fov_rad=math.radians(120.0),
                max_range_m=12000.0, p_detect=0.95, phase_s=0.25)
    base.update(kw)
    return CameraParams(sensor_id=sensor_id, seed=seed, **base)


def _straight_from(x: float, y: float, heading_deg: float, speed: float, duration: float) -> Trajectory:
    h = math.radians(heading_deg)
    return Trajectory(TargetState(x, y, speed * math.cos(h), speed * math.sin(h)), [Straight(duration)])


def crossing_pair() -> MultiScenario:
    # Both pass through (3000, 2500) at t = 45 s, 30 deg apart.
    cx, cy, t_x, v = 3000.0, 2500.0, 45.0, 70.0
    targets = []
    for tid, heading in ((1, 15.0), (2, -15.0)):
        h = math.radians(heading)
        x0, y0 = cx - v * math.cos(h) * t_x, cy - v * math.sin(h) * t_x
        targets.append(Target(tid, _straight_from(x0, y0, heading, v, 90.0)))
    # An omnidirectional bearing sensor whose +/-pi seam (directly behind its
    # boresight) passes through the crossing point, so both targets cross the
    # seam while their tracks are close: the camera's wrapped innovation is
    # exercised exactly where association is hardest.
    cam_x, cam_y = -3000.0, 3000.0
    yaw = math.atan2(cy - cam_y, cx - cam_x) + math.pi
    camera = _camera(20, 3102, x_m=cam_x, y_m=cam_y, yaw_rad=math.remainder(yaw, 2 * math.pi),
                     fov_rad=2 * math.pi)
    return MultiScenario("crossing_pair", [_radar(10, 3101)], [camera], targets, 90.0)


def close_pass() -> MultiScenario:
    lane = 120.0
    targets = [
        Target(1, _straight_from(-1500.0, 5000.0, 0.0, 60.0, 90.0)),
        Target(2, _straight_from(3900.0, 5000.0 + lane, 180.0, 60.0, 90.0)),
        Target(3, Trajectory(TargetState(-4000.0, -2500.0, 0.0, 80.0),
                             [Straight(25.0), Turn(30.0, -math.radians(3.0)), Straight(35.0)])),
    ]
    camera = _camera(21, 4102, x_m=-3000.0, y_m=0.0, yaw_rad=0.6)  # looks at the lanes
    return MultiScenario("close_pass", [_radar(11, 4101)], [camera], targets, 90.0,
                         start_ns=1_700_000_000 * 1_000_000_000)


def enter_leave() -> MultiScenario:
    radar = _radar(12, 5101, max_range_m=6000.0, p_detect=0.85, clutter_per_scan=1.0)
    camera = _camera(22, 5102, dropouts=((40.0, 46.0),))
    targets = [
        # Inbound from outside coverage: enters at ~6 km around t = 14 s.
        Target(1, _straight_from(7000.0, 1000.0, 180.0, 70.0, 120.0)),
        # Outbound: leaves coverage around t = 60 s.
        Target(2, _straight_from(-1000.0, -2000.0, -100.0, 70.0, 120.0)),
        # Appears at t = 30 s (e.g. take-off) inside coverage.
        Target(3, _straight_from(-3000.0, 2000.0, 20.0, 50.0, 90.0), start_s=30.0),
        # Vanishes at t = 70 s (e.g. landing) inside coverage.
        Target(4, _straight_from(1500.0, 4500.0, -90.0, 40.0, 70.0)),
    ]
    return MultiScenario("enter_leave", [radar], [camera], targets, 120.0)


def formation_split() -> MultiScenario:
    lane, v = 45.0, 80.0
    targets = []
    # Flying east, north of the radar, so the lanes are separated along range
    # (the radar's most precise direction, 10 m) and the formation stays well
    # inside coverage throughout.
    for tid, (dy, turn_deg) in ((1, (lane, 2.0)), (2, (0.0, 0.0)), (3, (-lane, -2.0))):
        legs = [Straight(40.0)]
        if turn_deg:
            legs += [Turn(10.0, math.radians(turn_deg)), Straight(40.0)]
        else:
            legs += [Straight(50.0)]
        targets.append(Target(tid, Trajectory(TargetState(-3500.0, 4500.0 + dy, v, 0.0), legs)))
    camera = _camera(23, 6102, x_m=-6000.0, y_m=0.0, yaw_rad=math.radians(40.0))
    return MultiScenario("formation_split", [_radar(13, 6101)], [camera], targets, 90.0)


def demo() -> MultiScenario:
    targets = []
    # The crossing pair: through (2500, 3000) at t = 60 s, 30 deg apart.
    cx, cy, t_x, v = 2500.0, 3000.0, 60.0, 60.0
    for tid, heading in ((1, 15.0), (2, -15.0)):
        h = math.radians(heading)
        targets.append(Target(tid, _straight_from(cx - v * math.cos(h) * t_x,
                                                  cy - v * math.sin(h) * t_x, heading, v, 150.0)))
    # Inbound from beyond the 8 km coverage edge (enters at ~t = 20 s)...
    targets.append(Target(3, _straight_from(9500.0, -2000.0, 170.0, 80.0, 150.0)))
    # ...outbound, leaving coverage at ~t = 95 s...
    targets.append(Target(4, _straight_from(-1000.0, -1500.0, -120.0, 70.0, 150.0)))
    # ...and a turning one.
    targets.append(Target(5, Trajectory(TargetState(-5000.0, 1000.0, 70.0, 0.0),
                                        [Straight(50.0), Turn(30.0, -math.radians(3.0)),
                                         Straight(70.0)])))
    camera = _camera(24, 7102, x_m=-4000.0, y_m=4000.0, yaw_rad=-math.pi / 4,
                     fov_rad=math.radians(150.0))
    return MultiScenario("demo", [_radar(14, 7101)], [camera], targets, 150.0)


G0_MPS2 = 9.80665  # standard gravity


@dataclass(frozen=True)
class ManoeuvreLeg:
    """One turn of a manoeuvre profile: load factor [g] (> 0) and direction."""

    load_g: float
    right: bool = True


def manoeuvre(load_g: float, speed_mps: float, seed: int, *, legs: tuple[ManoeuvreLeg, ...] = (),
              straight_s: float = 30.0, max_turn_s: float = 40.0,
              start: tuple[float, float] = (-6000.0, 15000.0)) -> MultiScenario:
    """Straight, turn(s), straight; see the module docstring.

    `legs` replaces the single right turn of `load_g` g with a sequence of
    turns, each followed by `straight_s` of straight flight (used for the
    separate tuning set: left turns and S-turns at other loads).
    """
    if not (load_g > 0.0 and speed_mps > 0.0):
        raise ValueError("load and speed must be positive")
    turns = legs or (ManoeuvreLeg(load_g),)
    segments: list = [Straight(straight_s)]
    for leg in turns:
        rate = leg.load_g * G0_MPS2 / speed_mps
        segments.append(Turn(min(math.pi / rate, max_turn_s), -rate if leg.right else rate))
        segments.append(Straight(straight_s))
    traj = Trajectory(TargetState(start[0], start[1], speed_mps, 0.0), segments)
    radar = _radar(1, seed, max_range_m=60000.0)
    name = f"manoeuvre_{load_g:g}g_{speed_mps:g}mps_s{seed}"
    return MultiScenario(name, [radar], [], [Target(1, traj)], traj.duration_s)


SCENARIOS = {s.__name__: s for s in (straight_crossing, turn)}
MULTI_SCENARIOS = {s.__name__: s for s in (crossing_pair, close_pass, enter_leave,
                                                 formation_split, demo)}
