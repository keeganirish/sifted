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

    def test_decode_round_trips_the_golden_fixture(self) -> None:
        golden = bytes.fromhex((FIXTURES / "measurement_bearing_only_v1.hex").read_text())
        m = wire.decode(golden)
        self.assertEqual((m.sensor_id, m.kind, m.seq, m.time_ns), (7, wire.BEARING_ONLY, 3, 10**9))
        self.assertEqual(wire.encode(m), golden)
        rb = wire.Measurement(2, wire.RANGE_BEARING, 9, -5, (1000.0, -0.5, 0.0), (10.0, 0.01, 0.0))
        self.assertEqual(wire.decode(wire.encode(rb)), rb)

    def test_decode_rejects_malformed_frames(self) -> None:
        good = bytes.fromhex((FIXTURES / "measurement_bearing_only_v1.hex").read_text())

        def patched(offset: int, value: bytes) -> bytes:
            return good[:offset] + value + good[offset + len(value):]

        bad = [good[:7], good[:-1], good + b"\0", patched(0, b"SIFX"), patched(4, b"\x02"),
               patched(5, b"\x02"), patched(6, b"\x41\x00"), patched(11, b"\x01"),
               patched(10, b"\x09"),
               patched(32, b"\x00" * 7 + b"\x80")]  # z[1] = -0.0 on a bearing-only frame
        for frame in bad:
            with self.assertRaises(ValueError, msg=frame.hex()):
                wire.decode(frame)


if __name__ == "__main__":
    unittest.main()
