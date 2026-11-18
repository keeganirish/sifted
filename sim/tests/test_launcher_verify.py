"""What Verify means for a run that never shut down cleanly (ADR 0012)."""

from __future__ import annotations

import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "app"))

import launcher  # noqa: E402


class CompareIncomplete(unittest.TestCase):
    def compare(self, live: bytes, replay: bytes, before_finish: int) -> dict:
        with tempfile.TemporaryDirectory() as tmp:
            a, b = pathlib.Path(tmp, "live.txt"), pathlib.Path(tmp, "replay.txt")
            a.write_bytes(live)
            b.write_bytes(replay)
            return launcher.compare_incomplete(a, b, before_finish)

    def test_live_output_cut_short_by_the_crash_matches(self) -> None:
        r = self.compare(b"a\nb\n", b"a\nb\nc\nfinal\n", 3)
        self.assertEqual((r["state"], r["lines"]), ("partial", 2))

    def test_a_torn_last_line_is_not_compared(self) -> None:
        r = self.compare(b"a\nb\npartly wri", b"a\nb\nc\n", 3)
        self.assertEqual((r["state"], r["lines"]), ("partial", 2))

    def test_the_replays_final_flush_is_not_compared(self) -> None:
        # The crashed run saved more than the recording holds; the flush must not count.
        r = self.compare(b"a\nb\nc\nd\n", b"a\nb\nFLUSHED\n", 2)
        self.assertEqual((r["state"], r["lines"]), ("partial", 2))

    def test_a_real_difference_is_reported(self) -> None:
        self.assertEqual(self.compare(b"a\nX\n", b"a\nb\nc\n", 3)["state"], "different")


class RunClassification(unittest.TestCase):
    def test_only_a_finished_run_ended_cleanly(self) -> None:
        self.assertTrue(launcher.ended_cleanly({"complete": True}))  # recordings from before
        self.assertFalse(launcher.ended_cleanly({"complete": False}))
        self.assertFalse(launcher.ended_cleanly({"complete": True, "crashed": True}))
        self.assertFalse(launcher.ended_cleanly({"complete": True, "log_failed": True}))

    def test_recovered_line_count_is_read_from_the_daemon(self) -> None:
        text = "sifted_fusiond: recovered replay: 76 output lines before final flush\n"
        self.assertEqual(launcher.recovered_lines(text), 76)
        self.assertIsNone(launcher.recovered_lines("sifted_fusiond: frames 3"))


if __name__ == "__main__":
    unittest.main()
