"""Checks that the Stone Soup baseline is configured like Sifted (README.md, "Configuration mapping").

    build/stonesoup-venv/bin/python -m unittest discover -s benchmarks/baselines/stonesoup

Skipped cleanly when Stone Soup is not installed. Not part of ctest or CI:
the baseline is optional developer tooling.
"""

from __future__ import annotations

import math
import pathlib
import sys
import unittest

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "sim"))

try:
    import numpy as np
    import stonesoup  # noqa: F401
    import tracker
except ImportError as e:  # pragma: no cover - depends on the environment
    raise unittest.SkipTest(f"Stone Soup baseline dependencies not installed ({e})")

from stonesoup.models.measurement.nonlinear import Cartesian2DToBearing, CartesianToBearingRange
from stonesoup.models.transition.linear import (CombinedLinearGaussianTransitionModel,
                                                ConstantVelocity)
from stonesoup.types.array import StateVector
from stonesoup.types.state import State

FIXTURES = REPO / "tests" / "fixtures" / "scenarios"


class ConfigurationMapping(unittest.TestCase):
    def test_versions_are_the_pinned_ones(self):
        pins = dict(line.split("==") for line in (HERE / "requirements.txt").read_text().splitlines()
                    if "==" in line and not line.startswith("#"))
        self.assertEqual(stonesoup.__version__, pins["stonesoup"])

    def test_measurement_models_match_sifted(self):
        # Sifted (radar.py, src/sifted/estimation/range_bearing.hpp):
        #   range = |p - s|, bearing = wrap(atan2(dy, dx) - yaw).
        sx, sy, yaw = -3000.0, 3000.0, 0.6
        for x, y in ((1200.0, 4100.0), (-3100.0, 2990.0), (5000.0, -7000.0)):
            want_b = math.remainder(math.atan2(y - sy, x - sx) - yaw, 2 * math.pi)
            state = State(StateVector([x, 10.0, y, -5.0]))
            rb = CartesianToBearingRange(ndim_state=4, mapping=(0, 2), noise_covar=np.eye(2),
                                         translation_offset=StateVector([sx, sy]),
                                         rotation_offset=StateVector([0.0, 0.0, yaw]))
            z = rb.function(state)
            self.assertAlmostEqual(float(z[0]), want_b, places=12)
            self.assertAlmostEqual(float(z[1]), math.hypot(x - sx, y - sy), places=8)
            b = Cartesian2DToBearing(ndim_state=4, mapping=(0, 2), noise_covar=np.eye(1),
                                     translation_offset=StateVector([sx, sy]),
                                     rotation_offset=StateVector([0.0, 0.0, yaw]))
            self.assertAlmostEqual(float(b.function(state)[0]), want_b, places=12)

    def test_process_noise_matches_sifted(self):
        # Sifted cv_filter.hpp: Q_axis = q [dt^3/3, dt^2/2; dt^2/2, dt], q = accel_psd.
        q, dt_s = tracker.Config().accel_psd, 0.75
        import datetime as dt
        model = CombinedLinearGaussianTransitionModel([ConstantVelocity(q), ConstantVelocity(q)])
        Q = np.asarray(model.covar(time_interval=dt.timedelta(seconds=dt_s)))
        axis = q * np.array([[dt_s ** 3 / 3, dt_s ** 2 / 2], [dt_s ** 2 / 2, dt_s]])
        np.testing.assert_allclose(Q[:2, :2], axis, rtol=1e-12)
        np.testing.assert_allclose(Q[2:, 2:], axis, rtol=1e-12)
        self.assertEqual(float(np.abs(Q[:2, 2:]).max()), 0.0)

    def test_gates_match_sifted(self):
        # Sifted's gates (engine.hpp AssociationConfig): 10.83 (1-D), 13.82 (2-D).
        from scipy.stats import chi2
        self.assertAlmostEqual(float(chi2.ppf(0.999, 1)), 10.8276, places=4)
        self.assertAlmostEqual(float(chi2.ppf(0.999, 2)), 13.8155, places=4)

    def test_defaults_are_sifteds(self):
        c = tracker.Config()
        self.assertEqual((c.confirm_hits, c.tentative_timeout_s, c.coast_after_s, c.drop_after_s,
                          c.max_position_sigma_m, c.max_tracks, c.initial_velocity_sigma),
                         (3, 3.0, 2.5, 10.0, 1000.0, 1024, 100.0))


class Tracking(unittest.TestCase):
    def test_tracks_a_fixture_deterministically_in_the_report_format(self):
        a, stats = tracker.track(FIXTURES / "close_pass")
        b, _ = tracker.track(FIXTURES / "close_pass")
        self.assertEqual(a, b)
        lines = a.splitlines()
        self.assertEqual(lines[0], tracker.REPORT_HEADER)
        self.assertEqual(lines[1], tracker.REPORT_COLUMNS)
        keys = [(int(r.split(",")[0]), int(r.split(",")[1])) for r in lines[2:]]
        self.assertEqual(keys, sorted(set(keys)))  # sorted, no duplicates
        self.assertGreaterEqual(stats.confirmed, 3)  # three targets
        self.assertTrue(all(len(r.split(",")) == 18 for r in lines[2:]))

    def test_never_reads_truth_positions(self):
        # Same instants, positions scrambled: identical output.
        import shutil
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            dst = pathlib.Path(d) / "s"
            shutil.copytree(FIXTURES / "close_pass", dst)
            rows = (dst / "truth.csv").read_text().splitlines()
            scrambled = [rows[0]] + [",".join([r.split(",")[0], "0", "0", "0", "0"] +
                                              r.split(",")[5:]) for r in rows[1:]]
            (dst / "truth.csv").write_text("\n".join(scrambled) + "\n")
            self.assertEqual(tracker.track(dst)[0], tracker.track(FIXTURES / "close_pass")[0])


if __name__ == "__main__":
    unittest.main()
