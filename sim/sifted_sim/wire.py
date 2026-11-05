"""Sifted wire format v1 encoder/decoder (Python mirror of src/sifted/wire).

Normative description: docs/adr/0001-wire-format.md. The shared golden fixture
in tests/fixtures/ keeps this and the C++ codec byte-for-byte in agreement.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

MAGIC = b"SIFT"
VERSION = 1
MSG_MEASUREMENT = 1
FRAME_SIZE = 72  # 8-byte header + 64-byte measurement payload

RANGE_BEARING, BEARING_ONLY, GEO_POSITION = 1, 2, 3
_USED = {RANGE_BEARING: 2, BEARING_ONLY: 1, GEO_POSITION: 3}


@dataclass(frozen=True)
class Measurement:
    sensor_id: int
    kind: int
    seq: int
    time_ns: int
    z: tuple[float, float, float]
    sigma: tuple[float, float, float]

    def __post_init__(self) -> None:
        # Unused components must be exactly 0.0, as the receiver requires.
        for i in range(_USED[self.kind], 3):
            if self.z[i] != 0.0 or self.sigma[i] != 0.0:
                raise ValueError("unused components must be 0.0")


def encode(m: Measurement) -> bytes:
    payload = struct.pack("<HBBIq3d3d", m.sensor_id, m.kind, 0, m.seq, m.time_ns, *m.z, *m.sigma)
    return MAGIC + struct.pack("<BBH", VERSION, MSG_MEASUREMENT, len(payload)) + payload
