import json
import math
import pathlib
import unittest

from sifted_sim import adsb, wire

SAMPLE = pathlib.Path(__file__).resolve().parent / "samples" / "opensky_states_sample.json"


def sample() -> dict:
    return json.loads(SAMPLE.read_text())


class AdsbTest(unittest.TestCase):
    def test_sample_converts_valid_airborne_states_only(self) -> None:
        a = adsb.Adapter(sensor_id=100)
        ms = a.convert(sample())
        # 10 rows: 4 good (one uses baro altitude as fallback for geo), and one
        # each on the ground, without position, stale, bad icao24, bad
        # longitude and without any altitude.
        self.assertEqual(len(ms), 4)
        self.assertEqual(dict(a.dropped), {"on_ground": 1, "no_position": 1, "stale": 1,
                                           "bad_icao24": 1, "bad_position": 1, "no_altitude": 1})
        times = [m.time_ns for m in ms]
        self.assertEqual(times, sorted(times))
        self.assertEqual([m.seq for m in ms], [0, 1, 2, 3])
        for m in ms:
            self.assertEqual((m.sensor_id, m.kind), (100, wire.GEO_POSITION))
            self.assertTrue(-90 <= m.z[0] <= 90 and -180 <= m.z[1] <= 180)
            self.assertEqual(len(wire.encode(m)), wire.FRAME_SIZE)

    def test_measurement_values_and_noise(self) -> None:
        ms = adsb.Adapter(sensor_id=7, sigma_m=30.0).convert(sample())
        ryr = next(m for m in ms if m.z[1] == -0.4521)
        self.assertEqual(ryr.time_ns, 1763199999 * 1_000_000_000)
        self.assertEqual(ryr.z, (51.4706, -0.4521, 3124.2))  # geo_altitude preferred
        self.assertAlmostEqual(ryr.sigma[0] * adsb.M_PER_DEG_LAT, 30.0)
        self.assertAlmostEqual(ryr.sigma[1] * adsb.M_PER_DEG_LAT * math.cos(math.radians(51.4706)), 30.0)
        dlh = next(m for m in ms if m.z[1] == 0.2611)
        self.assertEqual(dlh.z[2], 7802.9)

    def test_repeated_poll_sends_nothing_new_and_order_is_monotonic(self) -> None:
        a = adsb.Adapter(sensor_id=1)
        first = a.convert(sample())
        self.assertEqual(a.convert(sample()), [])  # same positions again
        self.assertEqual(a.dropped["repeat"], len(first))
        later = sample()
        later["time"] += 10
        for row in later["states"]:
            if isinstance(row[3], int):
                row[3] += 10
        newer = a.convert(later)
        self.assertGreater(len(newer), 0)
        self.assertGreaterEqual(newer[0].time_ns, first[-1].time_ns)
        self.assertEqual(newer[0].seq, len(first))

    def test_untrusted_shapes_are_rejected_not_crashing(self) -> None:
        a = adsb.Adapter(sensor_id=1)
        for bad in (None, [], "x", {"time": "now"}, {"time": 1, "states": "no"},
                    {"time": 1, "states": [[1, 2]]}, {"time": 1, "states": [None, 5, {}]},
                    {"time": float("nan"), "states": []}):
            self.assertEqual(a.convert(bad), [])
        self.assertEqual(a.convert({"time": 1, "states": None}), [])
        row = sample()["states"][0]
        row[8] = "false"  # on_ground must be a bool
        self.assertEqual(a.convert({"time": 1763200000, "states": [row]}), [])
        row = sample()["states"][0]
        row[3] = True  # bool is not an int timestamp here
        self.assertEqual(a.convert({"time": 1763200000, "states": [row]}), [])

    def test_quota_arithmetic(self) -> None:
        self.assertEqual(adsb.credit_cost(50, 52, -1, 1), 1)
        self.assertEqual(adsb.credit_cost(40, 50, -5, 5), 2)
        self.assertEqual(adsb.credit_cost(-90, 90, -180, 180), 4)
        self.assertAlmostEqual(adsb.min_interval_s(1, authenticated=False), 216.0)
        self.assertAlmostEqual(adsb.min_interval_s(1, authenticated=True), 21.6)
        self.assertEqual(adsb.min_interval_s(1, authenticated=True) >= adsb.AUTH_RESOLUTION_S, True)


if __name__ == "__main__":
    unittest.main()
