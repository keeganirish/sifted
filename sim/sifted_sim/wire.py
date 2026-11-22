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


_PAYLOAD = struct.Struct("<HBBIq3d3d")


def encode(m: Measurement) -> bytes:
    payload = _PAYLOAD.pack(m.sensor_id, m.kind, 0, m.seq, m.time_ns, *m.z, *m.sigma)
    return MAGIC + struct.pack("<BBH", VERSION, MSG_MEASUREMENT, len(payload)) + payload


def decode(frame: bytes) -> Measurement:
    """The measurement in a v1 frame; ValueError unless the frame is well formed.

    The structural checks of the C++ decoder (src/sifted/wire/codec.cpp), in
    its order: size, magic, version, message type, payload length, reserved
    byte, sensor kind, and unused components exactly +0.0 (bit pattern). Value
    ranges (finite numbers, positive sigmas) are left to the reader, as the
    C++ side leaves them to is_valid().
    """
    if len(frame) < 8:
        raise ValueError("truncated frame")
    if frame[:4] != MAGIC:
        raise ValueError("bad magic")
    version, msg, length = struct.unpack_from("<BBH", frame, 4)
    if version != VERSION:
        raise ValueError(f"unsupported version {version}")
    if msg != MSG_MEASUREMENT:
        raise ValueError(f"unknown message type {msg}")
    if length != _PAYLOAD.size or len(frame) != 8 + length:
        raise ValueError("length mismatch")
    sid, kind, reserved, seq, t, *rest = _PAYLOAD.unpack_from(frame, 8)
    if reserved != 0:
        raise ValueError("non-zero reserved byte")
    if kind not in _USED:
        raise ValueError(f"unknown sensor kind {kind}")
    used = _USED[kind]
    for i in range(used, 3):  # canonical +0.0, not -0.0
        if struct.pack("<d", rest[i]) != bytes(8) or struct.pack("<d", rest[3 + i]) != bytes(8):
            raise ValueError("unused components must be +0.0")
    return Measurement(sid, kind, seq, t, tuple(rest[:3]), tuple(rest[3:]))
