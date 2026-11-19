"""Property tests for the OpenSky state-vector CSV reader (threat model B6:
untrusted third-party files), in the style of test_adsb_fuzz.py. Standard
library only: a seeded generator builds random and mutated files; for every
one the reader either raises FormatError (a whole-file defect) or yields only
reports that satisfy its documented bounds, every one of which becomes a frame
the C++ decoder accepts. Cleaning then accounts for every report exactly once.
"""

import io
import math
import random
import unittest

from sifted_sim import adsb, realclean, statevectors as sv
try:  # unittest discover -s tests, or python -m unittest tests.<module>
    from test_adsb_fuzz import valid_frame
except ImportError:
    from tests.test_adsb_fuzz import valid_frame

ITERATIONS = 1500
HEADER = ["time", "icao24", "lat", "lon", "velocity", "heading", "vertrate", "callsign",
          "onground", "alert", "spi", "squawk", "baroaltitude", "geoaltitude", "lastposupdate",
          "lastcontact", "serials"]


def field(rng: random.Random, name: str, t: int) -> str:
    pick = rng.randrange(40)
    if pick == 0:
        return ""
    if pick == 1:
        return rng.choice(["nan", "inf", "-inf", "1e400", "-0.0", "0x10", "1_000", "١٢"])
    if pick == 2:
        return rng.choice(["True", "False", "true", "0", "1", "yes", ""])
    if pick == 3:
        return "".join(rng.choice('0123456789abcdef.,-"\x00 é') for _ in range(rng.randrange(10)))
    if pick == 4:
        return str(rng.choice([0, -1, 2 ** 63, 2 ** 64, 10 ** 30, t, t + 3600, t - 3600]))
    # Plausible values most of the time, so deep checks are reached.
    if name in ("time", "lastposupdate", "lastcontact"):
        return f"{t + rng.uniform(-20, 20):.{rng.randrange(4)}f}"
    if name == "icao24":
        return rng.choice(["4ca7b5", "ABCDEF", "abc12", "abc1234", "zzzzzz", "000000"])
    if name == "lat":
        return repr(rng.uniform(-95, 95))
    if name == "lon":
        return repr(rng.uniform(-185, 185))
    if name in ("baroaltitude", "geoaltitude"):
        return repr(rng.uniform(-2000, 120000))
    if name == "onground":
        return rng.choice(["True", "False"])
    if name == "velocity":
        return repr(rng.uniform(-10, 1200))
    if name == "heading":
        return repr(rng.uniform(-400, 800))
    return repr(rng.uniform(-1e3, 1e3))


def random_file(rng: random.Random) -> bytes:
    header = list(HEADER)
    rng.shuffle(header)
    if rng.random() < 0.05:
        header.remove(rng.choice(sv.REQUIRED))
    if rng.random() < 0.05:
        header.append(rng.choice(header))
    lines = [",".join(header)]
    t = 1_700_000_000
    for _ in range(rng.randrange(40)):
        t += rng.randrange(3)
        fields = [field(rng, h, t) for h in header]
        if rng.random() < 0.1:
            fields = fields[:rng.randrange(len(fields))]
        line = ",".join(f'"{f}"' if "," in f else f for f in fields)
        if rng.random() < 0.05:
            line = line[:rng.randrange(len(line) + 1)]
        lines.append(line)
    data = "\n".join(lines).encode("utf-8", "replace")
    if rng.random() < 0.7:
        data += b"\n"
    if rng.random() < 0.1:  # byte-level mutation
        b = bytearray(data)
        for _ in range(rng.randrange(1, 6)):
            if b:
                b[rng.randrange(len(b))] = rng.randrange(256)
        data = bytes(b)
    return data


class StateVectorFuzz(unittest.TestCase):
    def test_random_files_never_crash_and_yield_valid_reports(self) -> None:
        rng = random.Random(0x5EED_CAFE)
        reports_seen = 0
        for _ in range(ITERATIONS):
            data = random_file(rng)
            rd = sv.Reader()
            try:
                reports = list(rd.read_stream(io.BytesIO(data)))
            except sv.FormatError:
                continue
            self.assertEqual(len(reports) + sum(rd.dropped.values()), rd.rows)
            for r in reports:
                reports_seen += 1
                self.assertTrue(adsb._is_hex24(r.icao24) and r.icao24 == r.icao24.lower())
                self.assertTrue(-90 <= r.lat <= 90 and -180 <= r.lon <= 180)
                self.assertTrue(0 < r.t_ms <= adsb.MAX_TIME_POSITION_S * 1000)
                self.assertLessEqual(r.t_ms - r.row_time_s * 1000, sv.FUTURE_TOLERANCE_S * 1000 + 999)
                if r.alt_m is None:
                    self.assertTrue(r.on_ground)
                else:
                    self.assertTrue(adsb.MIN_ALT_M <= r.alt_m <= adsb.MAX_ALT_M)
                if r.speed_mps is not None:
                    self.assertTrue(0 <= r.speed_mps <= 1000 and 0 <= r.track_deg < 360)
                if r.alt_m is not None:
                    m = adsb.geo_measurement(100, reports_seen, r.t_ms * 1_000_000, r.lat, r.lon,
                                             r.alt_m)
                    self.assertTrue(valid_frame(m), r)
            flights, removed = realclean.clean("D1", reports)
            self.assertEqual(sum(len(f.reports) for f in flights) + sum(removed.values()),
                             len(reports))
        self.assertGreater(reports_seen, 200)  # the generator reaches the deep checks

    def test_line_and_row_limits_hold(self) -> None:
        rd = sv.Reader()
        data = b"time,icao24,lat,lon,onground,lastposupdate\n" + b"9" * (3 * sv.MAX_LINE_BYTES) + b"\n"
        self.assertEqual(list(rd.read_stream(io.BytesIO(data))), [])
        self.assertEqual(dict(rd.dropped), {"malformed": 1})
        self.assertTrue(math.isfinite(sv.MAX_FILE_BYTES))


if __name__ == "__main__":
    unittest.main()
