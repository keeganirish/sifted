"""Facts stated in more than one place in the repository must agree.

The version appears in CMakeLists.txt, app/launcher.py and sim/pyproject.toml.
The launcher's ADS-B tracker settings are repeated in docs/development.md for
people who run the daemon by hand. Reading the files as text keeps this test
standard-library only and independent of the build.
"""

import ast
import pathlib
import re
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[2]


def _read(rel):
    return (ROOT / rel).read_text(encoding="utf-8")


def _launcher_constant(name):
    tree = ast.parse(_read("app/launcher.py"))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in app/launcher.py")


class RepoConsistency(unittest.TestCase):
    def test_version_matches_everywhere(self):
        cmake = re.search(r"project\(sifted\s+VERSION\s+([0-9.]+)", _read("CMakeLists.txt"))
        pyproject = re.search(r'^version\s*=\s*"([^"]+)"', _read("sim/pyproject.toml"), re.M)
        self.assertIsNotNone(cmake, "VERSION not found in CMakeLists.txt project()")
        self.assertIsNotNone(pyproject, "version not found in sim/pyproject.toml")
        self.assertEqual(_launcher_constant("VERSION"), cmake.group(1))
        self.assertEqual(pyproject.group(1), cmake.group(1))

    def test_documented_adsb_tracker_args_match_launcher(self):
        args = _launcher_constant("ADSB_TRACKER_ARGS")
        docs = " ".join(_read("docs/development.md").replace("\\\n", " ").split())
        expected = " ".join(args)
        self.assertIn(expected, docs,
                      "docs/development.md must show the launcher's ADS-B tracker "
                      "settings (app/launcher.py ADSB_TRACKER_ARGS) in the same order")


if __name__ == "__main__":
    unittest.main()
