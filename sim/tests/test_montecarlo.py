"""Monte Carlo set generator (sifted_sim.montecarlo): seed scheme and sets."""

import pathlib
import random
import tempfile
import unittest

from sifted_sim import montecarlo, scenarios
from sifted_sim.montecarlo import EVENTS_XOR, plan, run_seed, sensor_seed


class SeedDerivation(unittest.TestCase):
    def test_golden_values_are_stable(self):
        # Changing these changes every Monte Carlo run: a deliberate decision.
        self.assertEqual(montecarlo.fnv1a64(""), 0xCBF29CE484222325)
        self.assertEqual(montecarlo.fnv1a64("a"), 0xAF63DC4C8601EC8C)
        self.assertEqual(run_seed(20261119, "close_pass", 0), 7121044663271466836)
        self.assertEqual(run_seed(20261119, "close_pass", 1), 6807558179343577089)

    def test_seeds_depend_on_every_input(self):
        s = run_seed(1, "close_pass", 0)
        self.assertNotEqual(s, run_seed(2, "close_pass", 0))
        self.assertNotEqual(s, run_seed(1, "crossing_pair", 0))
        self.assertNotEqual(s, run_seed(1, "close_pass", 1))
        self.assertNotEqual(sensor_seed(s, 10), sensor_seed(s, 11))
        self.assertTrue(0 <= s < 2 ** 64)

    def test_no_stream_collides_across_the_default_sets(self):
        # Every sensor's two random.Random seeds (noise, events), over all runs.
        seen = set()
        count = 0
        for r in plan():
            if r.set != "noise":
                continue  # the sweep reuses the noise seeds by design (CRN)
            sc = r.build()
            for p in list(sc.radars) + list(sc.cameras):
                seen.update((p.seed, p.seed ^ EVENTS_XOR))
                count += 2
        self.assertEqual(len(seen), count)

    def test_neighbouring_runs_are_uncorrelated(self):
        n = 1000
        a = [random.Random(sensor_seed(run_seed(7, "close_pass", i), 11)).random() for i in range(n)]
        b = [random.Random(sensor_seed(run_seed(7, "close_pass", i + 1), 11)).random()
             for i in range(n)]
        ma, mb = sum(a) / n, sum(b) / n
        cov = sum((x - ma) * (y - mb) for x, y in zip(a, b)) / n
        va = sum((x - ma) ** 2 for x in a) / n
        vb = sum((y - mb) ** 2 for y in b) / n
        r = cov / (va * vb) ** 0.5
        self.assertLess(abs(r), 4.0 / n ** 0.5)  # 4 standard errors under independence
        self.assertLess(abs(ma - 0.5), 4.0 * (1 / 12 / n) ** 0.5)


class Sets(unittest.TestCase):
    def test_default_plan_sizes(self):
        runs = plan()
        n_scen = len(scenarios.MULTI_SCENARIOS)
        noise = [r for r in runs if r.set == "noise"]
        sweep = [r for r in runs if r.set == "sweep"]
        self.assertEqual(len(noise), n_scen * montecarlo.NOISE_RUNS)
        conds = {r.condition for r in sweep}
        self.assertEqual(len(conds), len(montecarlo.SWEEP_CLUTTER) * len(montecarlo.SWEEP_PD))
        self.assertIn("clutter100_pd0.7", conds)
        for c in conds:
            self.assertEqual(sum(r.condition == c for r in sweep), n_scen * montecarlo.SWEEP_RUNS)
        self.assertEqual(len({r.dir for r in runs}), len(runs))

    def test_common_random_numbers_across_conditions(self):
        runs = plan()
        by = {}
        for r in runs:
            by.setdefault((r.scenario, r.run), set()).add(r.seed)
        self.assertTrue(all(len(s) == 1 for s in by.values()))

    def test_sweep_replaces_radar_parameters_only(self):
        r = next(x for x in plan() if x.condition == "clutter50_pd0.7")
        sc = r.build()
        base = scenarios.MULTI_SCENARIOS[r.scenario]()
        for radar, orig in zip(sc.radars, base.radars):
            self.assertEqual((radar.p_detect, radar.clutter_per_scan), (0.7, 50.0))
            self.assertEqual(radar.sigma_range_m, orig.sigma_range_m)
            self.assertEqual(radar.max_range_m, orig.max_range_m)
        for cam, orig in zip(sc.cameras, base.cameras):
            self.assertEqual(cam.p_detect, orig.p_detect)
            self.assertNotEqual(cam.seed, orig.seed)
        for t, o in zip(sc.targets, base.targets):  # same truth
            self.assertEqual(t.target_id, o.target_id)
            self.assertEqual(t.state_at(30.0), o.state_at(30.0))

    def test_noise_runs_keep_the_scenario_parameters(self):
        r = plan()[0]
        sc, base = r.build(), scenarios.MULTI_SCENARIOS[r.scenario]()
        self.assertEqual(sc.radars[0].clutter_per_scan, base.radars[0].clutter_per_scan)
        self.assertNotEqual(sc.radars[0].seed, base.radars[0].seed)

    def test_writes_smoke_sets_deterministically_and_skips_when_current(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            ma = montecarlo.write_sets(pathlib.Path(a), smoke=True, jobs=1)
            mb = montecarlo.write_sets(pathlib.Path(b), smoke=True, jobs=2)
            self.assertEqual(ma.read_bytes(), mb.read_bytes())
            lines = ma.read_text().splitlines()
            self.assertEqual(lines[0], montecarlo.MANIFEST_HEADER)
            self.assertEqual(lines[3], montecarlo.MANIFEST_COLUMNS)
            for row in lines[4:]:
                d = row.split(",")[-1]
                for f in ("sensors.cfg", "frames.hex", "truth.csv"):
                    self.assertEqual((pathlib.Path(a) / d / f).read_bytes(),
                                     (pathlib.Path(b) / d / f).read_bytes())
            stamp = (pathlib.Path(a) / lines[4].split(",")[-1] / "frames.hex").stat().st_mtime_ns
            montecarlo.write_sets(pathlib.Path(a), smoke=True, jobs=1)
            self.assertEqual(
                (pathlib.Path(a) / lines[4].split(",")[-1] / "frames.hex").stat().st_mtime_ns, stamp)


if __name__ == "__main__":
    unittest.main()
