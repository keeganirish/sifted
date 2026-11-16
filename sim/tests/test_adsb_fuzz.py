"""Property tests for the OpenSky adapter (threat model B2: untrusted
third-party JSON). Standard library only: a seeded random generator builds
random and mutated responses; for every one, the adapter must not raise, and
everything it emits must be a frame the C++ decoder accepts (the same checks
as sifted::is_valid and the strict wire decoder).
"""

import json
import math
import pathlib
import random
import struct
import unittest

from sifted_sim import adsb, wire

SAMPLE = pathlib.Path(__file__).resolve().parent / "samples" / "opensky_states_sample.json"
ITERATIONS = 3000

# sifted::is_valid for GeoPosition (src/sifted/core/measurement.cpp).
MIN_ALT_M, MAX_ALT_M, MAX_SIGMA = -1000.0, 100000.0, 1.0e6
INT64_MIN, INT64_MAX = -(2 ** 63), 2 ** 63 - 1


def valid_frame(m: wire.Measurement) -> bool:
    lat, lon, alt = m.z
    ok = (m.kind == wire.GEO_POSITION and -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0
          and MIN_ALT_M <= alt <= MAX_ALT_M
          and all(0.0 < s <= MAX_SIGMA for s in m.sigma)
          and 0 <= m.seq <= 0xFFFFFFFF and INT64_MIN <= m.time_ns <= INT64_MAX
          and 0 <= m.sensor_id <= 0xFFFF)
    if not ok:
        return False
    frame = wire.encode(m)
    # The decoder re-checks the header and that values survive the round trip.
    _, _, _, _, t, *rest = struct.unpack_from("<HBBIq3d3d", frame, 8)
    return len(frame) == wire.FRAME_SIZE and t == m.time_ns and all(map(math.isfinite, rest))


def random_value(rng: random.Random, depth: int = 0):
    choice = rng.randrange(14)
    if choice == 0:
        return None
    if choice == 1:
        return rng.choice([True, False])
    if choice == 2:
        return rng.randint(-10, 10)
    if choice == 3:
        return rng.choice([0, 1, -1, 2 ** 31, 2 ** 63, -(2 ** 63) - 1, 10 ** 30, 1763200000])
    if choice == 4:
        return rng.uniform(-1e6, 1e6)
    if choice == 5:
        return rng.choice([math.nan, math.inf, -math.inf, -0.0, 5e-324, 1e308])
    if choice == 6:
        return rng.uniform(-200, 200)
    if choice == 7:
        return "".join(rng.choice("0123456789abcdefABCDEF xyz\x00") for _ in range(rng.randrange(8)))
    if choice == 8:
        return rng.choice(["4ca7b5", "ABCDEF", "", "4ca7b", "4ca7b5x"])
    if choice in (9, 10) and depth < 2:
        return [random_value(rng, depth + 1) for _ in range(rng.randrange(20))]
    if choice == 11 and depth < 2:
        return {str(rng.randrange(5)): random_value(rng, depth + 1) for _ in range(rng.randrange(4))}
    return rng.uniform(0, 15000)


def mutate_state(rng: random.Random, row: list) -> list:
    row = list(row)
    for _ in range(rng.randrange(1, 4)):
        op = rng.randrange(4)
        if op == 0 and row:
            row[rng.randrange(len(row))] = random_value(rng)
        elif op == 1 and row:
            del row[rng.randrange(len(row)):]
        elif op == 2:
            row.append(random_value(rng))
        else:
            i = rng.randrange(len(row)) if row else 0
            row.insert(i, random_value(rng))
    return row


class AdsbPropertyTest(unittest.TestCase):
    def check(self, adapter: adsb.Adapter, response) -> None:
        before = adapter.newest_ns
        out = adapter.convert(response)  # must never raise
        last = before
        for m in out:
            self.assertTrue(valid_frame(m), m)
            if last is not None:
                self.assertGreaterEqual(m.time_ns, last)  # never older than anything sent
            last = m.time_ns

    def test_mutated_sample_responses(self) -> None:
        rng = random.Random(20261115)
        base = json.loads(SAMPLE.read_text())
        adapter = adsb.Adapter(sensor_id=100)
        for _ in range(ITERATIONS):
            resp = json.loads(json.dumps(base))
            resp["time"] = rng.choice([resp["time"], resp["time"] + rng.randrange(60),
                                       random_value(rng)])
            states = resp["states"]
            for i in range(len(states)):
                if rng.random() < 0.5:
                    states[i] = mutate_state(rng, states[i])
            if rng.random() < 0.1:
                resp["states"] = random_value(rng)
            self.check(adapter, resp)

    def test_random_responses(self) -> None:
        rng = random.Random(7)
        adapter = adsb.Adapter(sensor_id=3)
        for _ in range(ITERATIONS):
            resp = random_value(rng)
            if rng.random() < 0.6:
                resp = {"time": random_value(rng),
                        "states": [[random_value(rng) for _ in range(rng.randrange(20))]
                                   for _ in range(rng.randrange(10))]}
            self.check(adapter, resp)

    def test_timestamps_beyond_int64_nanoseconds_are_dropped(self) -> None:
        # Regression: a time_position whose nanosecond value overflows int64
        # used to pass every check and then make wire.encode() raise.
        base = json.loads(SAMPLE.read_text())
        for t in (2 ** 62, 10 ** 12, adsb.MAX_TIME_POSITION_S + 1):
            resp = json.loads(json.dumps(base))
            resp["time"] = t
            for row in resp["states"]:
                row[3] = t
            adapter = adsb.Adapter(sensor_id=1)
            self.assertEqual(adapter.convert(resp), [])
            self.assertGreater(adapter.dropped["bad_position"], 0)
        resp = json.loads(json.dumps(base))
        resp["time"] = adsb.MAX_TIME_POSITION_S
        for row in resp["states"]:
            row[3] = adsb.MAX_TIME_POSITION_S
        self.check(adsb.Adapter(sensor_id=1), resp)

    def test_mutated_json_text(self) -> None:
        rng = random.Random(99)
        text = SAMPLE.read_bytes()
        adapter = adsb.Adapter(sensor_id=9)
        for _ in range(ITERATIONS):
            b = bytearray(text)
            for _ in range(rng.randrange(1, 8)):
                b[rng.randrange(len(b))] = rng.randrange(256)
            try:
                resp = json.loads(b.decode("utf-8", errors="replace"))
            except (ValueError, RecursionError):
                continue  # what fetch() would report as a bad response
            self.check(adapter, resp)


if __name__ == "__main__":
    unittest.main()
