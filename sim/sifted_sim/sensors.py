"""Sensor table: where each sensor is and which way it points.

Python side of the "sifted-sensors 1" text format; the C++ reader is
parse_sensor_table() in src/sifted/core/sensor.hpp and the decision record is
docs/adr/0003-sensor-table.md. Floats are written with repr(), the shortest
form that round-trips, which the C++ side reads back bit-exactly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from . import wire

HEADER = "sifted-sensors 1"
MAX_OFFSET_M = 1.0e7
_KIND_NAMES = {
    wire.RANGE_BEARING: "range_bearing",
    wire.BEARING_ONLY: "bearing_only",
    wire.GEO_POSITION: "geo_position",
}
_KINDS_BY_NAME = {v: k for k, v in _KIND_NAMES.items()}


@dataclass(frozen=True)
class SensorConfig:
    sensor_id: int
    kind: int
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self) -> None:
        if not 0 <= self.sensor_id <= 0xFFFF:
            raise ValueError("sensor_id must fit in u16")
        if self.kind not in _KIND_NAMES:
            raise ValueError(f"unknown sensor kind {self.kind}")
        if not (abs(self.x_m) <= MAX_OFFSET_M and abs(self.y_m) <= MAX_OFFSET_M):
            raise ValueError("sensor position out of range")
        if not -math.pi <= self.yaw_rad <= math.pi:
            raise ValueError("yaw must be in [-pi, pi]")


def format_table(sensors: list[SensorConfig]) -> str:
    """Canonical text: header, then one line per sensor in id order."""
    ids = [s.sensor_id for s in sensors]
    if len(set(ids)) != len(ids):
        raise ValueError("duplicate sensor id")
    lines = [HEADER]
    for s in sorted(sensors, key=lambda s: s.sensor_id):
        lines.append(
            f"sensor {s.sensor_id} {_KIND_NAMES[s.kind]} "
            f"{float(s.x_m)!r} {float(s.y_m)!r} {float(s.yaw_rad)!r}"
        )
    return "\n".join(lines) + "\n"


def parse_table(text: str) -> list[SensorConfig]:
    """Reads the same format (used by tests and tooling; the C++ parser is normative)."""
    sensors: list[SensorConfig] = []
    have_header = False
    for raw in text.splitlines():
        tok = raw.split()
        if not tok or tok[0].startswith("#"):
            continue
        if not have_header:
            if " ".join(tok) != HEADER:
                raise ValueError("missing sifted-sensors 1 header")
            have_header = True
            continue
        if tok[0] != "sensor" or len(tok) != 6 or tok[2] not in _KINDS_BY_NAME:
            raise ValueError(f"malformed line: {raw!r}")
        sensors.append(
            SensorConfig(int(tok[1]), _KINDS_BY_NAME[tok[2]], float(tok[3]), float(tok[4]),
                         float(tok[5]))
        )
    if not have_header:
        raise ValueError("missing sifted-sensors 1 header")
    return sensors
