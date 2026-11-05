import pathlib
import unittest

from sifted_sim import wire

FIXTURES = pathlib.Path(__file__).resolve().parents[2] / "tests" / "fixtures"


class WireTest(unittest.TestCase):
    def test_matches_golden_fixture(self) -> None:
        golden = bytes.fromhex((FIXTURES / "measurement_bearing_only_v1.hex").read_text())
        m = wire.Measurement(
            sensor_id=7, kind=wire.BEARING_ONLY, seq=3, time_ns=1_000_000_000,
            z=(1.0, 0.0, 0.0), sigma=(0.5, 0.0, 0.0),
        )
        self.assertEqual(wire.encode(m), golden)
        self.assertEqual(len(golden), wire.FRAME_SIZE)

    def test_rejects_non_canonical_unused_component(self) -> None:
        with self.assertRaises(ValueError):
            wire.Measurement(1, wire.BEARING_ONLY, 0, 0, (0.0, 1.0, 0.0), (0.1, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
