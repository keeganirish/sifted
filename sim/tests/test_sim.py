import math
import pathlib
import struct
import unittest

from sifted_sim import camera, generate, radar, scenarios, sensors, wire
from sifted_sim.trajectory import Straight, Target, TargetState, Trajectory, Turn

FIXTURES = pathlib.Path(__file__).resolve().parents[2] / "tests" / "fixtures" / "scenarios"


def decode(frame: bytes) -> wire.Measurement:
    """Test-only decoder; the normative one is the C++ codec."""
    assert frame[:4] == wire.MAGIC and len(frame) == wire.FRAME_SIZE
    version, mtype, length = struct.unpack_from("<BBH", frame, 4)
    assert (version, mtype, length) == (wire.VERSION, wire.MSG_MEASUREMENT, 64)
    sid, kind, reserved, seq, t, *rest = struct.unpack_from("<HBBIq3d3d", frame, 8)
    assert reserved == 0
    return wire.Measurement(sid, kind, seq, t, tuple(rest[:3]), tuple(rest[3:]))


class TrajectoryTest(unittest.TestCase):
    def test_straight_line(self) -> None:
        tr = Trajectory(TargetState(1.0, 2.0, 3.0, -4.0), [Straight(10.0)])
        self.assertEqual(tr.state_at(10.0), TargetState(31.0, -38.0, 3.0, -4.0))

    def test_turn_keeps_speed_and_rotates_heading(self) -> None:
        w = math.radians(3.0)
        tr = Trajectory(TargetState(0.0, 0.0, 60.0, 0.0), [Turn(30.0, -w), Straight(10.0)])
        end_turn = tr.state_at(30.0)
        self.assertAlmostEqual(math.hypot(end_turn.vx_mps, end_turn.vy_mps), 60.0, places=9)
        self.assertAlmostEqual(math.atan2(end_turn.vy_mps, end_turn.vx_mps), -math.pi / 2, places=9)
        r = 60.0 / w  # right turn from heading east: centre is due south
        self.assertAlmostEqual(end_turn.x_m, r, places=6)
        self.assertAlmostEqual(end_turn.y_m, -r, places=6)
        # Continuous across the segment boundary.
        after = tr.state_at(30.0 + 1e-6)
        self.assertAlmostEqual(after.y_m, end_turn.y_m - 60.0 * 1e-6, places=6)

    def test_rejects_time_outside(self) -> None:
        with self.assertRaises(ValueError):
            Trajectory(TargetState(0, 0, 0, 0), [Straight(1.0)]).state_at(1.5)


class RadarTest(unittest.TestCase):
    def test_wrap_angle(self) -> None:
        self.assertAlmostEqual(radar.wrap_angle(3 * math.pi / 2), -math.pi / 2)
        self.assertAlmostEqual(radar.wrap_angle(-3 * math.pi / 2), math.pi / 2)
        self.assertEqual(radar.wrap_angle(0.25), 0.25)

    def test_noise_is_standard_normal(self) -> None:
        n = radar.Normal(7)
        xs = [n.sample() for _ in range(20000)]
        mean = sum(xs) / len(xs)
        var = sum((x - mean) ** 2 for x in xs) / len(xs)
        self.assertLess(abs(mean), 0.03)
        self.assertLess(abs(var - 1.0), 0.05)

    def test_measurements_are_valid_and_regular(self) -> None:
        s = scenarios.turn()
        samples = radar.observe(s.radar, s.trajectory, s.start_ns)
        self.assertEqual(len(samples), 91)  # 90 s at 1 Hz, both ends included
        for i, x in enumerate(samples):
            m = x.measurement
            self.assertEqual(m.seq, i)
            self.assertEqual(m.time_ns, s.start_ns + i * 1_000_000_000)
            self.assertTrue(0.0 <= m.z[0] and -math.pi <= m.z[1] <= math.pi)

    def test_noise_statistics_match_parameters(self) -> None:
        s = scenarios.straight_crossing()
        samples = radar.observe(s.radar, s.trajectory)
        dr = [x.measurement.z[0] - radar.true_range_bearing(s.radar, x.truth)[0] for x in samples]
        rms = math.sqrt(sum(d * d for d in dr) / len(dr))
        self.assertLess(abs(rms - s.radar.sigma_range_m) / s.radar.sigma_range_m, 0.35)

    def test_straight_crossing_crosses_the_bearing_seam(self) -> None:
        s = scenarios.straight_crossing()
        b = [x.measurement.z[1] for x in radar.observe(s.radar, s.trajectory)]
        self.assertTrue(any(v > 3.0 for v in b) and any(v < -3.0 for v in b))


class SensorTableTest(unittest.TestCase):
    def test_round_trip_is_exact(self) -> None:
        table = [sensors.SensorConfig(3, wire.RANGE_BEARING, 0.1, -2.5e3, -math.pi),
                 sensors.SensorConfig(1, wire.BEARING_ONLY, 1e-17, 7.0, 0.3)]
        text = sensors.format_table(table)
        self.assertTrue(text.startswith("sifted-sensors 1\nsensor 1 bearing_only"))
        self.assertEqual(sensors.parse_table(text), sorted(table, key=lambda s: s.sensor_id))

    def test_rejects_bad_input(self) -> None:
        with self.assertRaises(ValueError):
            sensors.format_table([sensors.SensorConfig(1, 1, 0, 0, 0)] * 2)
        with self.assertRaises(ValueError):
            sensors.SensorConfig(1, 1, 0.0, 0.0, 4.0)
        with self.assertRaises(ValueError):
            sensors.parse_table("sensor 1 range_bearing 0 0 0\n")


def _still(tid: int, x: float, y: float, duration: float = 20.0, start: float = 0.0) -> Target:
    return Target(tid, Trajectory(TargetState(x, y, 0.0, 0.0), [Straight(duration)]), start)


class MultiTargetRadarTest(unittest.TestCase):
    def params(self, **kw) -> radar.RadarParams:
        base = dict(sensor_id=1, x_m=0.0, y_m=0.0, yaw_rad=0.0, sigma_range_m=10.0,
                    sigma_bearing_rad=0.003, rate_hz=1.0, seed=9)
        base.update(kw)
        return radar.RadarParams(**base)

    def test_scan_reports_in_bearing_order_not_target_order(self) -> None:
        targets = [_still(1, 0.0, 3000.0), _still(2, 3000.0, 0.0)]  # bearings pi/2, 0
        dets = radar.observe_targets(self.params(), targets, 0.0)
        self.assertEqual([d.target_id for d in dets], [2, 1])
        self.assertEqual([d.measurement.seq for d in dets], [0, 1])

    def test_coverage_lifetime_and_dropouts(self) -> None:
        targets = [_still(1, 3000.0, 0.0), _still(2, 9000.0, 0.0), _still(3, 0.0, 3000.0, 5.0, 10.0)]
        dets = radar.observe_targets(self.params(max_range_m=8000.0, dropouts=((3.0, 5.0),)),
                                     targets, 19.0)
        times = {d.measurement.time_ns // radar.NS_PER_S for d in dets}
        self.assertNotIn(3, times)
        self.assertNotIn(4, times)
        self.assertNotIn(2, {d.target_id for d in dets})  # out of range
        t3 = sorted(d.measurement.time_ns // radar.NS_PER_S for d in dets if d.target_id == 3)
        self.assertEqual((t3[0], t3[-1]), (10, 15))  # exists only from 10 s to 15 s

    def test_detection_probability_and_clutter_statistics(self) -> None:
        p = self.params(max_range_m=5000.0, p_detect=0.8, clutter_per_scan=2.0)
        dets = radar.observe_targets(p, [_still(1, 3000.0, 0.0, 2000.0)], 1999.0)
        hits = sum(d.target_id == 1 for d in dets)
        clutter = [d for d in dets if d.target_id is None]
        self.assertLess(abs(hits / 2000 - 0.8), 0.03)
        self.assertLess(abs(len(clutter) / 2000 - 2.0), 0.1)
        self.assertTrue(all(d.measurement.z[0] <= 5000.0 + 1e-9 for d in clutter))

    def test_failure_modes_do_not_change_the_noise_of_remaining_detections(self) -> None:
        tgt = [_still(1, 3000.0, 0.0)]
        full = radar.observe_targets(self.params(), tgt, 19.0)
        lossy = radar.observe_targets(self.params(p_detect=0.5), tgt, 19.0)
        by_time = {d.measurement.time_ns: d.measurement.z for d in full}
        self.assertTrue(lossy and len(lossy) < len(full))
        for d in lossy:
            self.assertEqual(d.measurement.z, by_time[d.measurement.time_ns])


class CameraTest(unittest.TestCase):
    def params(self, **kw) -> camera.CameraParams:
        base = dict(sensor_id=5, x_m=100.0, y_m=-200.0, yaw_rad=0.4,
                    sigma_bearing_rad=math.radians(0.1), rate_hz=2.0, seed=77)
        base.update(kw)
        return camera.CameraParams(**base)

    def test_bearing_model_and_noise(self) -> None:
        p = self.params()
        tgt = [_still(1, 3100.0, 2800.0, 500.0)]  # 45 deg from the camera
        dets = camera.observe_targets(p, tgt, 500.0)
        self.assertEqual(len(dets), 1001)  # 2 Hz, both ends
        err = [radar.wrap_angle(d.measurement.z[0] - (math.pi / 4 - 0.4)) for d in dets]
        rms = math.sqrt(sum(e * e for e in err) / len(err))
        self.assertLess(abs(rms - p.sigma_bearing_rad) / p.sigma_bearing_rad, 0.1)
        m = dets[0].measurement
        self.assertEqual((m.kind, m.z[1:], m.sigma[1:]), (wire.BEARING_ONLY, (0.0, 0.0), (0.0, 0.0)))

    def test_field_of_view_phase_and_dropout(self) -> None:
        p = self.params(yaw_rad=0.0, fov_rad=math.radians(90.0), phase_s=0.25, dropouts=((2.0, 3.0),))
        tgt = [_still(1, 3000.0, 0.0), _still(2, -3000.0, 0.0)]  # ahead, behind
        dets = camera.observe_targets(p, tgt, 5.0)
        self.assertEqual({d.target_id for d in dets}, {1})
        times = [d.measurement.time_ns for d in dets]
        self.assertEqual(times[0], 250_000_000)
        self.assertFalse(any(2_000_000_000 <= t < 3_000_000_000 for t in times))

    def test_rejects_bad_parameters(self) -> None:
        with self.assertRaises(ValueError):
            self.params(sigma_bearing_rad=0.0)
        with self.assertRaises(ValueError):
            self.params(phase_s=0.6)  # longer than the 0.5 s frame period


class GenerateTest(unittest.TestCase):
    def test_render_is_deterministic(self) -> None:
        for make in list(scenarios.SCENARIOS.values()) + list(scenarios.MULTI_SCENARIOS.values()):
            self.assertEqual(generate.render(make()), generate.render(make()))

    def test_multi_scenario_frames_are_time_ordered_and_truth_is_layout_2(self) -> None:
        for name, make in scenarios.MULTI_SCENARIOS.items():
            with self.subTest(scenario=name):
                out = generate.render(make())
                frames = [decode(bytes.fromhex(h)) for h in out["frames.hex"].split()]
                times = [m.time_ns for m in frames]
                self.assertEqual(times, sorted(times))
                self.assertEqual({m.kind for m in frames}, {wire.RANGE_BEARING, wire.BEARING_ONLY})
                lines = out["truth.csv"].splitlines()
                self.assertEqual(lines[0], generate.TRUTH_HEADER_V2)
                self.assertTrue(lines[0].startswith(generate.TRUTH_HEADER_V1))
                ids = {int(row.split(",")[5]) for row in lines[1:]}
                self.assertGreaterEqual(len(ids), 2)

    def test_enter_leave_exercises_coverage_changes(self) -> None:
        rows = generate.render(scenarios.enter_leave())["truth.csv"].splitlines()[1:]
        covered: dict[int, list[int]] = {}
        for row in rows:
            f = row.split(",")
            covered.setdefault(int(f[5]), []).append(int(f[6]))
        self.assertEqual(covered[1][0], 0)   # starts outside, enters
        self.assertEqual(covered[1][-1], 1)
        self.assertEqual(covered[2][0], 1)   # starts inside, leaves
        self.assertEqual(covered[2][-1], 0)

    def test_committed_fixtures_match_generator(self) -> None:
        # Bytes can differ across platforms in the last ulp (libm sin/cos/atan2),
        # so compare decoded values with a tight tolerance rather than bytes.
        for name, make in {**scenarios.SCENARIOS, **scenarios.MULTI_SCENARIOS}.items():
            with self.subTest(scenario=name):
                fresh = generate.render(make())
                d = FIXTURES / name
                self.assertEqual((d / "sensors.cfg").read_text(), fresh["sensors.cfg"])
                old = (d / "frames.hex").read_text().split()
                new = fresh["frames.hex"].split()
                self.assertEqual(len(old), len(new))
                for a, b in zip(old, new):
                    ma, mb = decode(bytes.fromhex(a)), decode(bytes.fromhex(b))
                    self.assertEqual((ma.sensor_id, ma.seq, ma.time_ns), (mb.sensor_id, mb.seq, mb.time_ns))
                    for u, v in zip(ma.z + ma.sigma, mb.z + mb.sigma):
                        self.assertTrue(math.isclose(u, v, rel_tol=1e-12, abs_tol=1e-12))
                self.assertEqual(len((d / "truth.csv").read_text().splitlines()),
                                 len(fresh["truth.csv"].splitlines()))


class ManoeuvreScenarioTest(unittest.TestCase):
    """The parametric manoeuvre family and its Monte Carlo sets (ADR 0014)."""

    def test_turn_has_the_requested_load_and_duration(self) -> None:
        for g, v in ((0.3, 100.0), (5.0, 250.0)):
            s = scenarios.manoeuvre(g, v, 101)
            traj = s.targets[0].trajectory
            straight1, turn, straight2 = traj.segments
            rate = g * scenarios.G0_MPS2 / v
            self.assertAlmostEqual(turn.rate_rad_s, -rate)  # right turn
            self.assertAlmostEqual(turn.duration_s, min(math.pi / rate, 40.0))
            self.assertEqual((straight1.duration_s, straight2.duration_s), (30.0, 30.0))
            mid = traj.state_at(30.0 + turn.duration_s / 2)
            self.assertAlmostEqual(math.hypot(mid.vx_mps, mid.vy_mps), v)  # constant speed
            r = s.radars[0]
            self.assertEqual((r.max_range_m, r.p_detect, r.clutter_per_scan, r.seed),
                             (60000.0, 0.9, 0.5, 101))

    def test_sets_use_disjoint_seeds_and_profiles(self) -> None:
        from sifted_sim import manoeuvres
        self.assertFalse(set(manoeuvres.EVALUATION_SEEDS) & set(manoeuvres.TUNING_SEEDS))
        self.assertFalse(set(manoeuvres.EVALUATION_LOADS) & set(manoeuvres.TUNING_LOADS))
        names = [n for n, _ in manoeuvres.evaluation_set()]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn("manoeuvre_3g_250mps_s101", names)
        reseeded = manoeuvres.reseeded_set()
        base = scenarios.formation_split().radars[0].seed
        seeds = {s.radars[0].seed for n, s in reseeded if n.startswith("formation_split_")}
        self.assertNotIn(base, seeds)
        self.assertEqual(len(seeds), len(manoeuvres.RESEEDED_SEEDS))


if __name__ == "__main__":
    unittest.main()
