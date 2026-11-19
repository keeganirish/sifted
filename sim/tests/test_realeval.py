"""Real-data evaluation pipeline (sifted_sim.realeval and its modules), offline.

Every input here is SYNTHETIC: small hand-checkable rows, or a generated
OpenSky-format archive of made-up aircraft. Nothing reads the network or the
cached real data. The end-to-end class runs the C++ tools and is skipped
unless SIFTED_BIN_DIR names a directory holding sifted_realeval and
sifted_eval (ctest's realdata_e2e_python sets it).
"""

from __future__ import annotations

import ast
import hashlib
import io
import json
import math
import os
import pathlib
import random
import sys
import tarfile
import tempfile
import unittest

from sifted_sim import geo, realclean, realeval, recorded, statevectors as sv, wire
from sifted_sim.radar import RadarParams, observe_targets
from sifted_sim.trajectory import Target

HEADER = ("time_utc,time,icao24,lat,lon,velocity,heading,vertrate,callsign,onground,alert,spi,"
          "squawk,baroaltitude,geoaltitude,lastposupdate,lastcontact,serials,hour")


def row(t: float, icao: str = "abc123", lat: float = 50.0, lon: float = 8.0, *, pos_t=None,
        speed="200.0", heading="90.0", ground="False", alt="10000.0", time_s=None) -> str:
    pos_t = t if pos_t is None else pos_t
    time_s = int(t) if time_s is None else time_s
    return (f"x,{time_s},{icao},{lat!r},{lon!r},{speed},{heading},0.0,CS,{ground},False,False,,"
            f"{alt},{alt},{pos_t},{pos_t},\"[1, 2]\",0")


def csv_bytes(rows: list[str], header: str = HEADER) -> bytes:
    return ("\n".join([header] + rows) + "\n").encode()


def report(t_s: float, lat: float, lon: float, icao: str = "abc123", ground: bool = False,
           speed: float | None = 200.0, track: float | None = 90.0) -> sv.Report:
    return sv.Report(icao, round(t_s * 1000), int(t_s), lat, lon, 1000.0, speed, track, 0.0, ground)


# Metres per degree near 50 N, close enough for building test paths.
M_LAT = 111_229.0
M_LON = 71_696.0


def straight(n: int, *, dt: float = 1.0, t0: float = 1_700_000_000.0, v_east: float = 200.0,
             icao: str = "abc123", lat0: float = 50.0, lon0: float = 8.0) -> list[sv.Report]:
    return [report(t0 + i * dt, lat0, lon0 + v_east * i * dt / M_LON, icao) for i in range(n)]


class SecondsTest(unittest.TestCase):
    def test_exact_millisecond_parsing(self) -> None:
        self.assertEqual(sv.parse_seconds_ms("1792331998.998"), 1792331998998)
        self.assertEqual(sv.parse_seconds_ms("1792331998"), 1792331998000)
        self.assertEqual(sv.parse_seconds_ms("1.0005"), 1001)   # half up
        self.assertEqual(sv.parse_seconds_ms("1.0004"), 1000)
        self.assertEqual(sv.parse_seconds_ms("1.9996"), 2000)
        for bad in ("", "-1", "1e9", "nan", "1.", ".5", "1,5", " 1"):
            self.assertIsNone(sv.parse_seconds_ms(bad), bad)


class ReaderTest(unittest.TestCase):
    def test_valid_rows_and_every_drop_reason(self) -> None:
        rows = [
            row(100.0),                                      # ok
            row(101.0, pos_t="100.5"),                       # ok: position 0.5 s old
            row(102.0, icao="xyz"),                          # bad_icao24
            row(103.0).replace(",50.0,", ",,", 1),          # no_position
            row(104.0, lat=95.0),                            # bad_position
            row(105.0, lat=float("nan")),                    # bad_position (nan)
            row(106.0, pos_t="abc"),                         # bad_time
            row(107.0, pos_t="120.0"),                       # future (13 s ahead)
            row(140.0, pos_t="110.0"),                       # stale (30 s old)
            row(108.0, ground="maybe"),                      # malformed bool
            row(109.0, alt=""),                              # no_altitude (airborne)
            row(110.0, ground="True", alt=""),               # ok: ground without altitude
            row(111.0, speed="", heading=""),                # ok: no velocity
            row(112.0, speed="nan"),                         # ok, velocity unusable
            "x,1,2,3",                                       # malformed: field count
        ]
        reports, rd = sv.read_bytes(csv_bytes(rows))
        self.assertEqual(rd.rows, len(rows))
        self.assertEqual(dict(rd.dropped), {"bad_icao24": 1, "no_position": 1, "bad_position": 2,
                                            "bad_time": 1, "future": 1, "stale": 1,
                                            "malformed": 2, "no_altitude": 1})
        self.assertEqual([r.t_ms for r in reports], [100000, 100500, 110000, 111000, 112000])
        self.assertIsNone(reports[2].alt_m)
        self.assertTrue(reports[2].on_ground)
        self.assertIsNone(reports[3].speed_mps)
        self.assertIsNone(reports[4].speed_mps)
        self.assertEqual(reports[0].track_deg, 90.0)

    def test_columns_found_by_name_in_any_order(self) -> None:
        legacy = "lastposupdate,icao24,onground,lon,lat,time"
        data = csv_bytes(["1.5,ABC123,false,8.0,50.0,2"], legacy)
        reports, rd = sv.read_bytes(data)
        self.assertEqual(len(reports), 0)  # no altitude column at all: airborne -> dropped
        self.assertEqual(dict(rd.dropped), {"no_altitude": 1})
        data = csv_bytes(["1.5,ABC123,true,8.0,50.0,2"], legacy)
        reports, _ = sv.read_bytes(data)
        self.assertEqual((reports[0].icao24, reports[0].t_ms, reports[0].lon), ("abc123", 1500, 8.0))

    def test_truncated_last_line_is_dropped_not_parsed(self) -> None:
        data = csv_bytes([row(100.0), row(101.0)])[:-25]
        reports, rd = sv.read_bytes(data)
        self.assertEqual(len(reports), 1)
        self.assertEqual(dict(rd.dropped), {"malformed": 1})

    def test_bad_files_raise_format_error(self) -> None:
        for data in (b"", b"time,icao24,lat\n1,abc123,5\n", HEADER.encode() + b",lat\n"):
            with self.assertRaises(sv.FormatError):
                sv.read_bytes(data)

    def test_overlong_line_is_skipped(self) -> None:
        long_row = row(101.0) + "," + "9" * (sv.MAX_LINE_BYTES + 10)
        reports, rd = sv.read_bytes(csv_bytes([row(100.0), long_row, row(102.0)]))
        self.assertEqual([r.t_ms for r in reports], [100000, 102000])
        self.assertEqual(dict(rd.dropped), {"malformed": 1})

    def test_archive_members_are_read_without_extraction(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            path = pathlib.Path(d) / "x.tar.gz"
            write_tar(path, {"x/a_state_vectors_data4.csv": csv_bytes([row(100.0)]),
                             "x/README.txt": b"not csv", "../evil_state_vectors.csv": b"ignored?"})
            rd = sv.Reader()
            got = [(m, r.t_ms) for m, r in rd.read_path(str(path)) if m.startswith("x/")]
            self.assertEqual(got, [("x/a_state_vectors_data4.csv", 100000)])
            self.assertEqual(sorted(os.listdir(d)), ["x.tar.gz"])  # nothing extracted
            # Truncated archive: a clear FormatError, never a partial silent read.
            path.write_bytes(path.read_bytes()[:60])
            with self.assertRaises(sv.FormatError):
                list(sv.Reader().read_path(str(path)))
            # An archive without a state-vector member.
            write_tar(path, {"x/README.txt": b"hello"})
            with self.assertRaises(sv.FormatError):
                list(sv.Reader().read_path(str(path)))


def write_tar(path: pathlib.Path, members: dict[str, bytes]) -> None:
    with tarfile.open(path, "w:gz") as tar:
        for name, data in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mtime = 0
            tar.addfile(info, io.BytesIO(data))


class GeoTest(unittest.TestCase):
    def test_dateline_distance_is_short(self) -> None:
        d = geo.chord_m(10.0, 179.999, 10.0, -179.999)
        self.assertAlmostEqual(d, 0.002 * 111_320.0 * math.cos(math.radians(10.0)), delta=1.0)

    def test_enu_projection_and_velocity(self) -> None:
        e = geo.Enu(50.0, 8.0)
        x, y = e.project(50.0, 8.0)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        x, y = e.project(50.001, 8.0)
        self.assertAlmostEqual(x, 0.0, delta=1e-3)
        self.assertAlmostEqual(y, 111.2, delta=0.1)
        vx, vy = e.velocity(50.0, 8.0, 100.0, 90.0)  # due east at the origin
        self.assertAlmostEqual(vx, 100.0, places=9)
        self.assertAlmostEqual(vy, 0.0, places=9)
        # 1 deg of longitude east of the origin, "north" is rotated by meridian convergence.
        vx, vy = e.velocity(50.0, 9.0, 100.0, 0.0)
        self.assertAlmostEqual(math.degrees(math.atan2(vx, vy)), -math.sin(math.radians(50.0)), delta=0.01)


class CleaningTest(unittest.TestCase):
    def assert_conserved(self, reports, flights, removed) -> None:
        kept = sum(len(f.reports) for f in flights)
        self.assertEqual(kept + sum(removed.values()), len(reports))

    def test_ground_duplicates_repeats_and_one_outlier(self) -> None:
        rs = straight(400)
        rs[10] = report(rs[10].t_ms / 1000, rs[10].lat, rs[10].lon, ground=True)
        rs.insert(21, rs[20])  # duplicate_time
        rs.insert(31, report(rs[30].t_ms / 1000 + 0.5, rs[30].lat, rs[30].lon))  # repeated_position
        rs[50] = report(rs[50].t_ms / 1000, rs[50].lat + 0.05, rs[50].lon)  # 5.5 km jump
        flights, removed = realclean.clean("D1", rs)
        self.assertEqual(+removed, {"on_ground": 1, "duplicate_time": 1, "repeated_position": 1,
                                    "jump": 1})
        self.assertEqual(len(flights), 1)
        self.assert_conserved(rs, flights, removed)

    def test_jump_reset_restarts_from_the_rejected_run(self) -> None:
        # The first report is an outlier: the following ones are all "jumps" from
        # it, so after JUMP_RESET of them the run restarts from the newer reports.
        rs = straight(400)
        rs[0] = report(rs[0].t_ms / 1000, rs[0].lat + 0.2, rs[0].lon)
        flights, removed = realclean.clean("D1", rs)
        self.assertEqual(removed["jump"], realclean.JUMP_RESET - 1)
        self.assertEqual(removed["short_segment"], 1)  # the lone outlier segment
        self.assertEqual(len(flights), 1)
        self.assertEqual(flights[0].reports[0], rs[realclean.JUMP_RESET])
        self.assert_conserved(rs, flights, removed)

    def test_gap_splits_and_short_segments_drop(self) -> None:
        a = straight(400)
        b = straight(400, t0=a[-1].t_ms / 1000 + realclean.MAX_GAP_S + 1)  # same address, later
        c = straight(50, t0=b[-1].t_ms / 1000 + 600)                   # too short
        flights, removed = realclean.clean("D1", a + b + c)
        self.assertEqual([f.index for f in flights], [0, 1])
        self.assertEqual(+removed, {"short_segment": 50})
        self.assert_conserved(a + b + c, flights, removed)

    def test_two_transmitters_on_one_address_are_excluded(self) -> None:
        a = straight(400)
        b = straight(400, lat0=51.0, t0=a[0].t_ms / 1000 + 0.5)
        mixed = sorted(a + b, key=lambda r: r.t_ms)
        flights, removed = realclean.clean("D1", mixed)
        self.assertEqual(flights, [])
        self.assertEqual(+removed, {"inconsistent_identity": 800})

    def test_dateline_crossing_is_not_a_jump(self) -> None:
        rs = straight(400, lon0=179.0, lat0=0.0, v_east=250.0)
        rs = [report(r.t_ms / 1000, r.lat, ((r.lon + 180.0) % 360.0) - 180.0) for r in rs]
        self.assertTrue(any(r.lon < 0 for r in rs) and any(r.lon > 0 for r in rs))
        flights, removed = realclean.clean("D1", rs)
        self.assertEqual((len(flights), +removed), (1, {}))

    def test_chunks_respect_the_radius(self) -> None:
        f = realclean.clean("D1", straight(400, v_east=250.0))[0][0]  # 100 km
        chunks = realclean.chunks(f, radius_m=30_000.0)
        self.assertEqual(sum(len(c) for c in chunks), 400)
        for c in chunks:
            self.assertTrue(all(geo.chord_m(c[0].lat, c[0].lon, r.lat, r.lon) <= 30_000.0 for r in c))
        self.assertEqual(len(chunks), 4)


class InterpolationTest(unittest.TestCase):
    def knots(self, f, ts):
        return [recorded.Knot(t, *f(t)) for t in ts]

    def test_hermite_is_exact_for_constant_acceleration(self) -> None:
        def f(t):  # x = 3 + 4t + 0.5 t^2, y = -2t
            return 3 + 4 * t + 0.5 * t * t, -2 * t, 4 + t, -2.0
        traj = recorded.RecordedTrajectory(self.knots(f, [0.0, 2.0, 5.0, 9.0]), "hermite")
        for t in (0.0, 0.7, 2.0, 3.3, 8.9, 9.0):
            s = traj.state_at(t)
            x, y, vx, vy = f(t)
            for a, b in ((s.x_m, x), (s.y_m, y), (s.vx_mps, vx), (s.vy_mps, vy)):
                self.assertAlmostEqual(a, b, places=9)

    def test_linear_and_loo_on_constant_velocity(self) -> None:
        def f(t):
            return 10 * t, 5.0, 10.0, 0.0
        ks = self.knots(f, [0.0, 1.0, 2.5, 4.0])
        traj = recorded.RecordedTrajectory(ks, "linear")
        self.assertAlmostEqual(traj.state_at(3.0).x_m, 30.0)
        self.assertEqual(traj.duration_s, 4.0)
        for m in recorded.METHODS:
            errs = recorded.leave_one_out(ks, m)
            self.assertEqual([s for s, _ in errs], [2.5, 3.0])
            self.assertTrue(all(e < 1e-9 for _, e in errs))

    def test_missing_velocity_uses_finite_differences(self) -> None:
        ks = [recorded.Knot(0.0, 0.0, 0.0), recorded.Knot(1.0, 10.0, 0.0), recorded.Knot(2.0, 20.0, 0.0)]
        s = recorded.RecordedTrajectory(ks, "hermite").state_at(0.5)
        self.assertAlmostEqual(s.x_m, 5.0)
        self.assertAlmostEqual(s.vx_mps, 10.0)

    def test_rejects_bad_knots(self) -> None:
        with self.assertRaises(ValueError):
            recorded.RecordedTrajectory([recorded.Knot(0.0, 0, 0)])
        with self.assertRaises(ValueError):
            recorded.RecordedTrajectory([recorded.Knot(0.0, 0, 0), recorded.Knot(0.0, 1, 1)])
        with self.assertRaises(ValueError):
            recorded.RecordedTrajectory([recorded.Knot(1.0, 0, 0), recorded.Knot(2.0, 1, 1)])
        with self.assertRaises(ValueError):
            recorded.RecordedTrajectory([recorded.Knot(0.0, 0, 0), recorded.Knot(1.0, math.nan, 1)])
        traj = recorded.RecordedTrajectory([recorded.Knot(0.0, 0, 0), recorded.Knot(1.0, 1, 1)])
        with self.assertRaises(ValueError):
            traj.state_at(1.5)

    def test_recorded_target_is_observed_by_the_existing_radar(self) -> None:
        traj = recorded.RecordedTrajectory(self.knots(lambda t: (1000 + 50 * t, 2000.0, 50.0, 0.0),
                                                      [0.0, 4.0, 10.0]))
        radar = RadarParams(1, 0.0, 0.0, 0.0, 10.0, 0.003, 1.0, 7, max_range_m=10_000.0)
        dets = observe_targets(radar, [Target(1, traj, start_s=-2.0)], 8.0)
        self.assertEqual(len(dets), 9)  # t = 0..8 s, the path exists from -2 to 8 s
        self.assertTrue(all(d.target_id == 1 for d in dets))

    def test_sigma_estimate_recovers_known_noise(self) -> None:
        rng = random.Random(5)
        sigma = 10.0
        ks = [recorded.Knot(float(t), 200.0 * t + rng.gauss(0, sigma), rng.gauss(0, sigma), 200.0, 0.0)
              for t in range(4000)]
        errs = [e for s, e in recorded.leave_one_out(ks, "linear")]
        stats = {(0.0, realeval.SIGMA_SPACING_S): realeval.quant(errs)}
        self.assertAlmostEqual(realeval.estimate_sigma(stats), sigma, delta=0.5)


class WithholdingTest(unittest.TestCase):
    def test_schedule_is_hand_checkable(self) -> None:
        rs = tuple(straight(400))  # one report per second from t0
        mask = realeval.withheld_mask(rs, phase=0)
        withheld = [i for i, w in enumerate(mask) if w]
        b = int(realeval.A_BURN_IN_S)
        expected, start = [], b
        for g in realeval.A_GAPS_S:
            expected += list(range(start, min(start + int(g), 400)))
            start += int(g) + int(realeval.A_FEED_S)
            if start >= 400:
                break
        self.assertEqual(withheld, expected)
        self.assertEqual(withheld[:3], [b, b + 1, b + 62])  # 2 s gap, 60 s fed, then 5 s gap
        self.assertEqual(realeval.withheld_mask(rs, phase=0), mask)  # deterministic
        self.assertNotEqual(realeval.withheld_mask(rs, phase=1), mask)


def flight_infos(reports_by_icao: dict[str, list[sv.Report]]):
    ds = []
    for i, (icao, rs) in enumerate(sorted(reports_by_icao.items())):
        ds.append(realeval.Dataset(f"D{i + 1}", {}, rs, {}, len(rs), 0))
    return realeval.label_flights(ds)[0]


class IdentityTest(unittest.TestCase):
    def test_tracker_input_carries_no_identity(self) -> None:
        """Renaming every aircraft changes nothing the tracker reads, and the
        scoring files hold only integer labels."""
        def runs(names: tuple[str, str]):
            a = straight(500, icao=names[0])
            b = straight(500, icao=names[1], lat0=50.01, t0=1_700_000_000.4)
            return flight_infos({names[0]: a, names[1]: b})

        outputs = []
        # The third pair sorts the other way round, which swaps the integer labels.
        pairs = (("a1b2c3", "b4c5d6"), ("c7d8e9", "d1e2f3"), ("f9e8d7", "1a2b3c"))
        with tempfile.TemporaryDirectory() as d:
            for k, names in enumerate(pairs):
                for feed in ("full", "poll"):
                    fl = runs(names)
                    fl = [realeval.FlightInfo(f.name, "heldout", f.aircraft, f.flight) for f in fl]
                    (run,) = realeval.a_runs(fl)
                    self.assertEqual(len(run.members), 2)  # two aircraft share one run
                    out = pathlib.Path(d) / f"{k}_{feed}"
                    realeval.write_a_run(run, out, feed, withhold=True)
                    files = {p.name: p.read_bytes() for p in out.iterdir()}
                    outputs.append((feed, files))
                    for name in names:
                        for blob in files.values():
                            self.assertNotIn(name.encode(), blob)
                            self.assertNotIn(bytes.fromhex(name), blob)
                            self.assertNotIn(bytes.fromhex(name)[::-1], blob)
        by_feed = {}
        for feed, files in outputs:
            by_feed.setdefault(feed, []).append(files)
        for feed, (first, second, swapped) in by_feed.items():
            self.assertEqual(first, second, feed)  # every file, byte for byte
            frames = [bytes.fromhex(x) for x in first["frames.hex"].decode().split()]
            self.assertTrue(frames)
            self.assertTrue(all(len(f) == wire.FRAME_SIZE for f in frames))
            if feed == "full":
                # Labels follow address order, but the tracker's input does not.
                # (The poll feed goes through the live adapter, which orders
                # same-second reports by address, as it does live; the engine
                # assigns such a same-time group jointly.)
                self.assertEqual(first["frames.hex"], swapped["frames.hex"])
                self.assertNotEqual(first["feed_key.csv"], swapped["feed_key.csv"])

    def test_tracker_args_match_the_launcher(self) -> None:
        src = (pathlib.Path(__file__).resolve().parents[2] / "app" / "launcher.py").read_text()
        for node in ast.parse(src).body:
            if isinstance(node, ast.Assign) and any(
                    isinstance(t, ast.Name) and t.id == "ADSB_TRACKER_ARGS" for t in node.targets):
                self.assertEqual(ast.literal_eval(node.value), realeval.ADSB_TRACKER_ARGS)
                return
        self.fail("ADSB_TRACKER_ARGS not found in app/launcher.py")


class FailureTest(unittest.TestCase):
    def test_empty_after_filtering_fails_clearly(self) -> None:
        with self.assertRaisesRegex(realeval.EvalError, "no flight left after cleaning"):
            flight_infos({"abc123": straight(20)})

    def test_missing_and_corrupt_cache_fail_clearly(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            m = {"datasets": [{"label": "D1", "file": "x.tar.gz", "sha256": "0" * 64, "bytes": 3}]}
            with self.assertRaisesRegex(realeval.EvalError, "realeval fetch"):
                realeval.load_datasets(m, pathlib.Path(d))
            (pathlib.Path(d) / "x.tar.gz").write_bytes(b"abc")
            with self.assertRaisesRegex(realeval.EvalError, "SHA-256"):
                realeval.load_datasets(m, pathlib.Path(d))


# ---- end to end ---------------------------------------------------------------------

def synthetic_archive(path: pathlib.Path) -> None:
    """A made-up OpenSky-format archive: three aircraft, two of them crossing
    paths at the same time, with noise, a coverage gap, a ground segment,
    duplicates and junk rows. SYNTHETIC: not real aircraft."""
    rng = random.Random(20261119)
    rows = []

    def fly(icao, t0, n, lat0, lon0, v_e, v_n, turn_at=None):
        lat, lon, ve, vn = lat0, lon0, v_e, v_n
        for i in range(n):
            t = t0 + i
            if turn_at is not None and i >= turn_at:  # 1.5 deg/s left turn
                h = math.atan2(vn, ve) + math.radians(1.5)
                sp = math.hypot(ve, vn)
                ve, vn = sp * math.cos(h), sp * math.sin(h)
            lat += vn / M_LAT
            lon += ve / M_LON
            if 300 <= i < 330 and icao == "a00001":
                continue  # 30 s coverage hole
            noisy_lat = lat + rng.gauss(0.0, 8.0) / M_LAT
            noisy_lon = lon + rng.gauss(0.0, 8.0) / M_LON
            track = (math.degrees(math.atan2(ve, vn)) + 360.0) % 360.0
            pos_t = f"{t + rng.randrange(0, 900) / 1000:.3f}"
            rows.append(row(t, icao, noisy_lat, noisy_lon, pos_t=pos_t, speed=f"{math.hypot(ve, vn):.2f}",
                            heading=f"{track:.3f}", time_s=t + 1))
            if i % 7 == 0:  # a re-emitted state vector without a new position
                rows.append(row(t + 1, icao, noisy_lat, noisy_lon, pos_t=pos_t, time_s=t + 1))
    fly("a00001", 1_700_000_000, 800, 50.0, 8.0, 150.0, 0.0, turn_at=500)
    fly("b00002", 1_700_001_000, 700, 50.0, 8.6, 0.0, 120.0)
    fly("c00003", 1_700_001_010, 700, 50.3, 8.3, 110.0, -110.0, turn_at=350)
    rows.append(row(1_700_002_000, "c00003", 50.2, 8.2, ground="True", alt=""))
    rows += ["garbage", row(1_700_002_001, icao="zz")]
    write_tar(path, {"synthetic/synthetic_state_vectors_data4.csv": csv_bytes(rows)})


def bin_dir() -> pathlib.Path | None:
    d = os.environ.get("SIFTED_BIN_DIR")
    if d and all((pathlib.Path(d) / t).exists() for t in ("sifted_realeval", "sifted_eval")):
        return pathlib.Path(d)
    return None


@unittest.skipIf(bin_dir() is None, "SIFTED_BIN_DIR with sifted_realeval and sifted_eval not set")
class EndToEnd(unittest.TestCase):
    def test_pipeline_is_deterministic_and_complete(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            root = pathlib.Path(d)
            cache = root / "cache"
            cache.mkdir()
            arch = cache / "synthetic.tar.gz"
            synthetic_archive(arch)
            data = arch.read_bytes()
            rows = sv.Reader()
            reports = [r for _, r in rows.read_path(str(arch))]
            manifest = {"synthetic": True, "terms": "Synthetic test input.", "datasets": [{
                "label": "D1", "file": arch.name, "bytes": len(data),
                "sha256": hashlib.sha256(data).hexdigest(), "retrieved": "n/a",
                "licence": "synthetic", **realeval.describe(reports, rows.rows)}]}
            (root / "manifest.json").write_text(json.dumps(manifest))
            texts = []
            for k in range(2):  # the same command twice, as for the committed report
                out = root / "report.md"
                rc = realeval.main(["run", "--manifest", str(root / "manifest.json"), "--cache",
                                    str(cache), "--work", str(root / f"work{k}"), "--bin-dir",
                                    str(bin_dir()), "--out", str(out)])
                self.assertEqual(rc, 0)
                texts.append(out.read_text(encoding="utf-8"))
            self.assertEqual(texts[0], texts[1])
            text = texts[0]
            self.assertIn("SYNTHETIC FIXTURE", text)
            for icao in ("a00001", "b00002", "c00003"):
                self.assertNotIn(icao, text)
            for section in ("## Provenance", "## Cleaning", "## Results A", "## Results B",
                            "## Findings", "## Limitations", "## Reproduction"):
                self.assertIn(section, text)
            # Two aircraft shared one held-out run, so the multi-aircraft
            # continuity metrics were exercised on real tool output.
            all_case = realeval.parse_realeval(
                (root / "work0" / "A" / "heldout_all.txt").read_text(encoding="ascii"))
            self.assertEqual(all_case["aircraft"], 2)
            self.assertTrue(any(r["aircraft"] == "2" for r in all_case["run"].values()))
            self.assertGreater(all_case["confirmed_tracks"], 0)
            self.assertGreater(all_case["nis"]["n"], 0)
            withheld = realeval.parse_realeval(
                (root / "work0" / "A" / "heldout_withheld.txt").read_text(encoding="ascii"))
            self.assertGreater(withheld["horizon"][("tracker", "all", "all")]["n"], 0)
            self.assertIn("| heldout | all | real path |", text)


if __name__ == "__main__":
    unittest.main()
