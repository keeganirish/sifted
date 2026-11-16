"""Seeded sensor fault injection for simulated runs (M3).

Mirrors tests/faults.hpp (the C++ test helpers) on wire.Measurement lists in
send order, so the faults the tests characterise can also be shown live:

    python3 -m sifted_sim.send crossing_pair --fault restart:10@30 \
        --fault clock:20:+2 --fault stuck:10@40-48

Each fault touches one sensor and leaves the others alone:

    dropout:ID@A-B      nothing from ID for A <= t < B (seconds after the first frame)
    intermittent:ID:P   ID is off for whole 2 s periods, each with probability P
    restart:ID@A        ID's seq numbers start again from 0 at A (a reboot)
    clock:ID:OFF[:PPM]  ID stamps t + OFF s (+ PPM drift); it still sends at t
    bias:ID:DZ0[:DZ1]   constant bias on ID's first (and second) component
    stuck:ID@A-B        ID repeats its last scan before A, freshly stamped, until B

Expected system behaviour for each: docs/threat-model.md, "Sensor and input
faults". Randomness (intermittent) uses SplitMix64 seeded from --fault-seed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from . import wire
from .impair import SplitMix64

NS = 1_000_000_000


@dataclass(frozen=True)
class Sent:
    """A measurement and the (true) time it is sent at."""

    send_ns: int
    m: wire.Measurement


def _span(text: str) -> tuple[int, float, float]:
    sid, _, rest = text.partition("@")
    a, _, b = rest.partition("-")
    return int(sid), float(a), float(b) if b else math.inf


def apply(spec: str, items: list[Sent], seed: int = 1) -> list[Sent]:
    """Applies one fault spec (see the module docstring) and returns new items."""
    kind, _, arg = spec.partition(":")
    if not items:
        return items
    t0 = min(i.send_ns for i in items)

    def rel(i: Sent) -> float:
        return (i.m.time_ns - t0) / NS

    if kind == "dropout":
        sid, a, b = _span(arg)
        return [i for i in items if not (i.m.sensor_id == sid and a <= rel(i) < b)]
    if kind == "intermittent":
        sid_s, _, p = arg.partition(":")
        sid, p_off = int(sid_s), float(p)
        rng, off = SplitMix64(seed), {}
        out = []
        for i in items:
            if i.m.sensor_id == sid:
                slot = (i.m.time_ns - t0) // (2 * NS)
                if slot not in off:
                    off[slot] = rng.uniform() < p_off
                if off[slot]:
                    continue
            out.append(i)
        return out
    if kind == "restart":
        sid, a, _ = _span(arg)
        nxt, out = 0, []
        for i in items:
            if i.m.sensor_id == sid and rel(i) >= a:
                i = Sent(i.send_ns, replace(i.m, seq=nxt))
                nxt += 1
            out.append(i)
        return out
    if kind == "clock":
        parts = arg.split(":")
        sid, offset = int(parts[0]), round(float(parts[1]) * NS)
        ppm = float(parts[2]) if len(parts) > 2 else 0.0
        out = []
        for i in items:
            if i.m.sensor_id == sid:
                drift = int(ppm * 1e-6 * (i.m.time_ns - t0))
                i = Sent(i.send_ns, replace(i.m, time_ns=i.m.time_ns + offset + drift))
            out.append(i)
        return out
    if kind == "bias":
        parts = arg.split(":")
        sid, dz0 = int(parts[0]), float(parts[1])
        dz1 = float(parts[2]) if len(parts) > 2 else 0.0
        out = []
        for i in items:
            m = i.m
            if m.sensor_id == sid:
                z = list(m.z)
                z[0] += dz0
                if m.kind == wire.RANGE_BEARING:
                    z[1] = math.remainder(z[1] + dz1, 2 * math.pi)
                i = Sent(i.send_ns, replace(m, z=tuple(z)))
            out.append(i)
        return out
    if kind == "stuck":
        sid, a, b = _span(arg)
        frozen: list[wire.Measurement] = []
        for i in items:
            if i.m.sensor_id == sid and rel(i) < a:
                if frozen and frozen[0].time_ns != i.m.time_ns:
                    frozen = []
                frozen.append(i.m)
        out, seq, last = [], 1 << 29, None
        for i in items:
            if i.m.sensor_id != sid or not (a <= rel(i) < b) or not frozen:
                out.append(i)
                continue
            if i.m.time_ns == last:
                continue
            last = i.m.time_ns
            for f in frozen:
                out.append(Sent(i.send_ns, replace(f, time_ns=i.m.time_ns, seq=seq)))
                seq += 1
        return out
    raise ValueError(f"unknown fault {kind!r} (dropout, intermittent, restart, clock, bias, stuck)")
