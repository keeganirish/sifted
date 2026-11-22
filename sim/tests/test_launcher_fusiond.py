"""Which sifted_fusiond the launcher runs."""

from __future__ import annotations

import os
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "app"))

import launcher  # noqa: E402


class FindFusiond(unittest.TestCase):
    def test_a_requested_path_that_is_missing_is_not_replaced_by_another_build(self) -> None:
        with mock.patch.dict(os.environ, {}, clear=False):
            os.environ.pop("SIFTED_FUSIOND", None)
            self.assertIsNone(launcher.find_fusiond("/nonexistent/sifted_fusiond"))

    def test_the_environment_variable_counts_as_a_request(self) -> None:
        with mock.patch.dict(os.environ, {"SIFTED_FUSIOND": "/nonexistent/sifted_fusiond"}):
            self.assertIsNone(launcher.find_fusiond(None))
            self.assertEqual(launcher.requested_fusiond(None), "/nonexistent/sifted_fusiond")

    def test_a_requested_executable_is_used(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            exe = pathlib.Path(tmp, "sifted_fusiond")
            exe.write_text("#!/bin/sh\n")
            exe.chmod(0o755)
            self.assertEqual(launcher.find_fusiond(str(exe)), exe.resolve())


if __name__ == "__main__":
    unittest.main()
