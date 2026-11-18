"""Ground-truth overlay: parsing, time alignment, matching and projection."""

import pathlib
import re
import unittest

from sifted_sim import generate, send, truth
from sifted_sim.truth import TruthError

S = truth.NS_PER_S
V2 = generate.TRUTH_HEADER_V2
ORIGIN = {"lat": 51.4779, "lon": -0.0015, "alt": 0}


def csv_v2(*rows):
    return "\n".join([V2, *rows]) + "\n"


def request(time_ns, tracks=(), pairs=None):
    return truth.parse_compare_request({
        "time_ns": time_ns, "origin": ORIGIN, "pairs": pairs or {},
        "tracks": [{"id": i, "x": x, "y": y, "status": status} for i, x, y, status in tracks],
    })


class ParseTest(unittest.TestCase):
    def test_multi_target_layout(self):
        t = truth.parse_truth(csv_v2("0,1.5,2.5,3.0,4.0,7,1", "1000000000,2.5,3.5,3.0,4.0,7,0",
                                     "1000000000,9.0,9.0,0.0,0.0,8,1"))
        self.assertEqual(sorted(t.targets), [7, 8])
        self.assertEqual(t.targets[7].times_ns, [0, S])
        self.assertEqual(t.targets[7].covered, [True, False])
        self.assertEqual((t.start_ns, t.end_ns), (0, S))

    def test_single_target_layout_is_one_covered_target(self):
        t = truth.parse_truth(f"{generate.TRUTH_HEADER_V1}\n5,1.0,2.0,0.0,0.0\n6,1.5,2.0,0.0,0.0\n")
        self.assertEqual(list(t.targets), [1])
        self.assertEqual(t.targets[1].covered, [True, True])

    def test_every_simulated_scenario_parses_and_shares_the_frame_clock(self):
        for name, make in generate.ALL.items():
            with self.subTest(scenario=name):
                files = generate.render(make())
                t = truth.parse_truth(files["truth.csv"])
                frames = [send.frame_time_ns(bytes.fromhex(x)) for x in files["frames.hex"].split()]
                # Snapshot time is measurement time, so it must lie inside the truth's span.
                self.assertGreaterEqual(min(frames), t.start_ns)
                self.assertLessEqual(max(frames), t.end_ns)

    def test_malformed_files_are_rejected(self):
        for text in ("", "a,b,c\n1,2,3\n", V2 + "\n",
                     csv_v2("0,1.0,2.0,0.0,0.0,1"),               # missing a field
                     csv_v2("x,1.0,2.0,0.0,0.0,1,1"),             # bad time
                     csv_v2("0,nan,2.0,0.0,0.0,1,1"),             # non-finite
                     csv_v2("0,1.0,2.0,0.0,0.0,1,2"),             # bad flag
                     csv_v2("5,1.0,2.0,0.0,0.0,1,1", "5,1.0,2.0,0.0,0.0,1,1")):  # time repeats
            with self.subTest(text=text), self.assertRaises(TruthError):
                truth.parse_truth(text)


class InterpolationTest(unittest.TestCase):
    def setUp(self):
        self.target = truth.parse_truth(csv_v2(
            f"{10 * S},0.0,100.0,0.0,0.0,1,0",
            f"{11 * S},10.0,80.0,0.0,0.0,1,1",
            f"{13 * S},30.0,80.0,0.0,0.0,1,1")).targets[1]

    def test_linear_between_samples(self):
        x, y, _ = self.target.at(10 * S + S // 4)
        self.assertAlmostEqual(x, 2.5)
        self.assertAlmostEqual(y, 95.0)

    def test_uneven_sample_spacing(self):
        self.assertAlmostEqual(self.target.at(12 * S)[0], 20.0)

    def test_exact_at_the_first_and_last_sample(self):
        self.assertEqual(self.target.at(10 * S), (0.0, 100.0, False))
        self.assertEqual(self.target.at(13 * S), (30.0, 80.0, True))

    def test_never_extrapolated_before_appearing_or_after_leaving(self):
        self.assertIsNone(self.target.at(10 * S - 1))
        self.assertIsNone(self.target.at(13 * S + 1))

    def test_coverage_is_that_of_the_latest_sample(self):
        self.assertFalse(self.target.at(11 * S - 1)[2])
        self.assertTrue(self.target.at(11 * S)[2])


class MatchTest(unittest.TestCase):
    def test_one_to_one_minimum_total_distance(self):
        # Greedy would take the closest pair (2, 5) at 2 m, for a total of 52 m; the best is 48 m.
        m = truth.match_targets({1: (0.0, 0.0), 2: (10.0, 0.0)},
                                {5: (8.0, 0.0), 6: (50.0, 0.0)}, {})
        self.assertEqual(m, {1: 5, 2: 6})

    def test_pairs_beyond_the_match_distance_are_forbidden(self):
        d = truth.MATCH_DISTANCE_M
        self.assertEqual(truth.match_targets({1: (0.0, 0.0)}, {5: (d, 0.0)}, {}), {1: 5})
        self.assertEqual(truth.match_targets({1: (0.0, 0.0)}, {5: (d + 0.01, 0.0)}, {}), {})

    def test_extra_targets_and_tracks_stay_unmatched(self):
        m = truth.match_targets({1: (0.0, 0.0), 2: (5000.0, 0.0)},
                                {5: (1.0, 0.0), 6: (-9000.0, 0.0)}, {})
        self.assertEqual(m, {1: 5})

    def test_previous_pairing_is_kept_while_valid(self):
        # Track 6 is now closer to target 1, but track 5 is still within the distance.
        m = truth.match_targets({1: (0.0, 0.0)}, {5: (40.0, 0.0), 6: (1.0, 0.0)}, {1: 5})
        self.assertEqual(m, {1: 5})

    def test_previous_pairing_is_dropped_when_too_far_or_gone(self):
        targets = {1: (0.0, 0.0)}
        self.assertEqual(truth.match_targets(targets, {5: (500.0, 0.0), 6: (1.0, 0.0)}, {1: 5}), {1: 6})
        self.assertEqual(truth.match_targets(targets, {6: (1.0, 0.0)}, {1: 5}), {1: 6})

    def test_two_targets_crossing_keep_their_tracks(self):
        pairs = {}
        for step in range(-10, 11):
            # Targets swap sides at step 0; each track lags its own target by 15 m,
            # so near the crossing each track is closer to the other target.
            a, b = step * 10.0, -step * 10.0
            lag = 15.0 if step >= 0 else -15.0
            targets = {1: (a, 0.0), 2: (b, 0.0)}
            tracks = {5: (a - lag, 0.0), 6: (b + lag, 0.0)}
            pairs = {**pairs, **truth.match_targets(targets, tracks, pairs)}
            self.assertEqual(pairs, {1: 5, 2: 6}, f"step {step}")

    def test_without_history_the_same_crossing_would_swap(self):
        # Guards the test above: the geometry really is ambiguous at the crossing.
        m = truth.match_targets({1: (10.0, 0.0), 2: (-10.0, 0.0)},
                                {5: (-5.0, 0.0), 6: (5.0, 0.0)}, {})
        self.assertEqual(m, {1: 6, 2: 5})

    def test_many_targets_are_solved_exactly(self):
        n = 60
        targets = {i: (i * 300.0, 0.0) for i in range(n)}
        tracks = {1000 + i: (i * 300.0 + 3.0, 4.0) for i in range(n)}
        self.assertEqual(truth.match_targets(targets, tracks, {}), {i: 1000 + i for i in range(n)})

    def test_constants_mirror_the_scoring_rule(self):
        header = pathlib.Path(__file__).resolve().parents[2] / "eval/sifted/eval/scenario.hpp"
        text = header.read_text(encoding="utf-8")
        for name, value in (("kMatchDistanceM", truth.MATCH_DISTANCE_M),
                            ("kMissBurnInS", truth.MISS_BURN_IN_S)):
            found = re.search(rf"{name} = ([0-9.]+);", text)
            self.assertIsNotNone(found, name)
            self.assertEqual(float(found.group(1)), value, name)


class ProjectionTest(unittest.TestCase):
    def test_matches_the_daemons_projection(self):
        # (x, y, lat, lon) of tracks in a snapshot written by sifted_fusiond.
        origin = (51.4779, -0.0015, 0.0)
        for x, y, lat, lon in (
                (-4989.982933175436, 999.9236029540341, 51.48686547049682, -0.0733391668385586),
                (-954.7342834167066, 3925.5624726394776, 51.51318268669576, -0.015252916766890735),
                (-1004.4835585943873, -1488.7216592109849, 51.46451821785254, -0.0159541600498094)):
            got = truth.enu_to_geodetic(origin, x, y)
            self.assertAlmostEqual(got[0], lat, places=9)  # 1e-9 deg is about 0.1 mm
            self.assertAlmostEqual(got[1], lon, places=9)


class CompareTest(unittest.TestCase):
    def setUp(self):
        self.truth = truth.parse_truth(csv_v2(
            f"{10 * S},0.0,0.0,0.0,0.0,1,1", f"{20 * S},100.0,0.0,0.0,0.0,1,1",
            f"{10 * S},5000.0,0.0,0.0,0.0,2,0", f"{20 * S},5000.0,0.0,0.0,0.0,2,1",
            f"{18 * S},-3000.0,0.0,0.0,0.0,3,1", f"{20 * S},-3000.0,0.0,0.0,0.0,3,1"))

    def test_errors_states_and_mean(self):
        r = truth.compare(self.truth, request(15 * S, [(5, 50.0, 3.0, "confirmed"), (6, 1e4, 0.0, "coasting")]))
        by_id = {t["id"]: t for t in r["targets"]}
        self.assertEqual(by_id[1]["state"], "matched")
        self.assertEqual(by_id[1]["track"], 5)
        self.assertAlmostEqual(by_id[1]["error_m"], 3.0)
        self.assertEqual(by_id[2]["state"], "uncovered")  # unmatched, but nobody could track it
        self.assertNotIn(3, by_id)                        # has not appeared yet
        self.assertEqual(r["tracks"], {"5": {"target": 1, "error_m": 3.0}})
        self.assertEqual(r["unmatched_tracks"], [6])
        self.assertAlmostEqual(r["mean_error_m"], 3.0)
        self.assertEqual((r["matched"], r["missed"], r["present"]), (1, 0, 2))

    def test_a_covered_target_is_missed_only_after_the_burn_in(self):
        states = lambda t: {x["id"]: x["state"] for x in truth.compare(self.truth, request(t))["targets"]}
        self.assertEqual(states(14 * S)[1], "acquiring")
        self.assertEqual(states(15 * S)[1], "missed")
        self.assertEqual(states(19 * S)[3], "acquiring")  # appeared at 18 s

    def test_tentative_tracks_are_neither_matched_nor_flagged(self):
        r = truth.compare(self.truth, request(15 * S, [(5, 50.0, 0.0, "tentative")]))
        self.assertEqual(r["tracks"], {})
        self.assertEqual(r["unmatched_tracks"], [])

    def test_pairs_survive_a_gap_in_the_match(self):
        r = truth.compare(self.truth, request(15 * S, [(5, 50.0, 0.0, "confirmed")]))
        self.assertEqual(r["pairs"], {"1": 5})
        r = truth.compare(self.truth, request(16 * S, [], pairs=r["pairs"]))
        self.assertEqual(r["pairs"], {"1": 5})
        r = truth.compare(self.truth, request(17 * S, [(5, 70.0, 40.0, "coasting"), (9, 70.0, 0.0, "confirmed")],
                                              pairs=r["pairs"]))
        self.assertEqual(r["tracks"]["5"]["target"], 1)
        self.assertEqual(r["unmatched_tracks"], [9])

    def test_trail_covers_the_recent_samples_and_ends_at_the_target(self):
        r = truth.compare(self.truth, request(15 * S))
        target = next(t for t in r["targets"] if t["id"] == 1)
        self.assertEqual(len(target["trail"]), 2)  # the 10 s sample, then now
        self.assertAlmostEqual(target["trail"][-1][0], target["lat"], places=6)
        self.assertAlmostEqual(target["trail"][-1][1], target["lon"], places=6)

    def test_a_time_outside_the_truth_compares_nothing(self):
        r = truth.compare(self.truth, request(99 * S, [(5, 0.0, 0.0, "confirmed")], pairs={"1": 5}))
        self.assertFalse(r["in_span"])
        self.assertEqual((r["targets"], r["unmatched_tracks"]), ([], []))
        self.assertEqual(r["pairs"], {"1": 5})


class RequestValidationTest(unittest.TestCase):
    def valid(self):
        return {"time_ns": 1, "origin": dict(ORIGIN), "pairs": {"1": 2},
                "tracks": [{"id": 1, "x": 0.5, "y": 1, "status": "confirmed"}]}

    def test_accepts_a_good_request(self):
        r = truth.parse_compare_request(self.valid())
        self.assertEqual((r.tracks, r.pairs), ({1: (0.5, 1.0)}, {1: 2}))

    def test_rejects_bad_fields(self):
        cases = [("time_ns", "1"), ("time_ns", True), ("time_ns", 1.5), ("origin", None),
                 ("origin", {"lat": 91, "lon": 0}), ("origin", {"lat": float("nan"), "lon": 0}),
                 ("tracks", {}), ("tracks", [1]), ("tracks", [{"id": "a", "x": 0, "y": 0, "status": "confirmed"}]),
                 ("tracks", [{"id": 1, "x": float("inf"), "y": 0, "status": "confirmed"}]),
                 ("tracks", [{"id": 1, "x": 0, "y": 0}]),
                 ("pairs", []), ("pairs", {"a": 1}), ("pairs", {"1": "b"})]
        for key, value in cases:
            with self.subTest(key=key, value=value), self.assertRaises(TruthError):
                truth.parse_compare_request({**self.valid(), key: value})


if __name__ == "__main__":
    unittest.main()
