import math
import pathlib
import struct
import unittest

from sifted_sim import generate, radar, scenarios, sensors, wire
from sifted_sim.trajectory import Straight, TargetState, Trajectory, Turn

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


class GenerateTest(unittest.TestCase):
    def test_render_is_deterministic(self) -> None:
        for make in scenarios.SCENARIOS.values():
            self.assertEqual(generate.render(make()), generate.render(make()))

    def test_committed_fixtures_match_generator(self) -> None:
        # Bytes can differ across platforms in the last ulp (libm sin/cos/atan2),
        # so compare decoded values with a tight tolerance rather than bytes.
        for name, make in scenarios.SCENARIOS.items():
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


if __name__ == "__main__":
    unittest.main()
