"""Sends a scenario's frames to sifted_fusiond over UDP, paced by their timestamps.

    python3 -m sifted_sim.send crossing_pair                       # real time
    python3 -m sifted_sim.send ../tests/fixtures/scenarios/turn --speed 10
    python3 -m sifted_sim.send close_pass --speed 0                # as fast as possible
    python3 -m sifted_sim.send enter_leave --rebase-now            # timestamps from now

The source is a scenario name (generated in memory) or a directory holding a
frames.hex file (e.g. a committed fixture). The daemon needs the matching
sensor table: `python3 -m sifted_sim.generate <scenario> <dir>` writes it as
<dir>/sensors.cfg.

--rebase-now shifts every frame's measurement time so the first frame carries
the current wall-clock time. Use it when simulated sensors share a daemon with
live ADS-B, whose timestamps are wall-clock: the engine drops measurements
older than its clock (OutOfOrder).
"""

from __future__ import annotations

import argparse
import pathlib
import socket
import struct
import sys
import time

from . import generate, wire

_PAYLOAD = struct.Struct("<HBBIq3d3d")


def load_frames(source: str) -> list[bytes]:
    path = pathlib.Path(source)
    if (path / "frames.hex").is_file():
        text = (path / "frames.hex").read_text(encoding="ascii")
    elif source in generate.ALL:
        text = generate.render(generate.ALL[source]())["frames.hex"]
    else:
        raise SystemExit(f"send: {source!r} is neither a scenario name nor a directory with frames.hex")
    return [bytes.fromhex(line) for line in text.split()]


def frame_time_ns(frame: bytes) -> int:
    return _PAYLOAD.unpack_from(frame, 8)[4]


def with_time(frame: bytes, time_ns: int) -> bytes:
    """The same frame with its measurement time replaced (re-encoded, still canonical)."""
    sid, kind, _reserved, seq, _t, *rest = _PAYLOAD.unpack_from(frame, 8)
    m = wire.Measurement(sid, kind, seq, time_ns, tuple(rest[:3]), tuple(rest[3:]))
    return wire.encode(m)


def schedule(frames: list[bytes], speed: float, rebase_ns: int | None) -> list[tuple[float, bytes]]:
    """(seconds after start, datagram) for every frame."""
    if not frames:
        return []
    t0 = frame_time_ns(frames[0])
    out = []
    for f in frames:
        t = frame_time_ns(f)
        offset = 0.0 if speed <= 0 else (t - t0) / 1e9 / speed
        out.append((offset, with_time(f, rebase_ns + (t - t0)) if rebase_ns is not None else f))
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted_sim.send", description=__doc__.splitlines()[0])
    ap.add_argument("source", help="scenario name or directory containing frames.hex")
    ap.add_argument("--target", default="127.0.0.1:5600", help="daemon UDP address [127.0.0.1:5600]")
    ap.add_argument("--speed", type=float, default=1.0, help="time scale; 0 = no pacing [1]")
    ap.add_argument("--rebase-now", action="store_true", help="shift timestamps to the wall clock")
    args = ap.parse_args(argv)
    host, _, port = args.target.rpartition(":")
    frames = load_frames(args.source)
    rebase = time.time_ns() if args.rebase_now else None
    plan = schedule(frames, args.speed, rebase)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start = time.monotonic()
    for offset, datagram in plan:
        delay = start + offset - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        sock.sendto(datagram, (host, int(port)))
    print(f"sent {len(plan)} frames to {args.target}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
