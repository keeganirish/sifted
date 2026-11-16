"""End-to-end latency of the real daemon: UDP send -> snapshot on the WebSocket.

    python3 benchmarks/e2e_latency.py build/release/src/apps/sifted_fusiond \
        [--targets 10,100,1000] [--scans 30] [--windows 0,0.5] > e2e.csv

Starts sifted_fusiond on loopback with snapshots on every flush
(--snapshot-interval 0), subscribes to its WebSocket stream, and sends a
synthetic radar picture: every `period` seconds one scan of N detections (one
per target, 10 m / 0.2 deg noise, targets on straight lines; seeded). For
each scan, latency = arrival on the WebSocket of the LAST snapshot stamped
with the scan's time (a scan larger than max_scan_size is processed, and
published, in chunks; the last one includes the whole scan), minus the moment
the scan's first datagram was sent.
Both ends are timestamped with the same monotonic clock in this process.

What the number contains: UDP receive and validation, the queue, the
daemon's scan assembly (a radar scan with nothing after it is flushed by the
100 ms quiet tick, so ~100 ms is structural), the reorder hold
(--reorder-window), engine processing, JSON formatting and WebSocket
delivery to one local client. Python's own scheduling adds well under a
millisecond (it sends the whole scan before it starts timing the next).
Standard library only.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import os
import random
import socket
import statistics
import struct
import subprocess
import sys
import tempfile
import threading
import time

WIRE = struct.Struct("<4sBBHHBBIq3d3d")  # header + measurement payload (ADR 0001)


def frame(seq: int, t_ns: int, r: float, b: float) -> bytes:
    return WIRE.pack(b"SIFT", 1, 1, 64, 1, 1, 0, seq & 0xFFFFFFFF, t_ns, r, b, 0.0,
                     10.0, math.radians(0.2), 0.0)


def free_port(kind: int) -> int:
    s = socket.socket(socket.AF_INET, kind)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class WsReader(threading.Thread):
    """Minimal RFC 6455 client: handshake, then unmasked server text frames."""

    def __init__(self, port: int) -> None:
        super().__init__(daemon=True)
        self.sock = None
        for _ in range(100):
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
                break
            except OSError:
                time.sleep(0.05)
        if self.sock is None:
            raise SystemExit("e2e: cannot connect to the WebSocket")
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((f"GET / HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
                           f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            buf += self.sock.recv(4096)
        self.buf = buf.split(b"\r\n\r\n", 1)[1]
        self.snapshots: list[tuple[int, int]] = []  # (monotonic ns on arrival, time_ns)
        self.stop = False

    def _read(self, n: int) -> bytes:
        while len(self.buf) < n:
            chunk = self.sock.recv(1 << 20)
            if not chunk:
                raise EOFError
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def run(self) -> None:
        try:
            while not self.stop:
                b0, b1 = self._read(2)
                n = b1 & 0x7F
                if n == 126:
                    n = struct.unpack(">H", self._read(2))[0]
                elif n == 127:
                    n = struct.unpack(">Q", self._read(8))[0]
                payload = self._read(n)
                now = time.monotonic_ns()
                if b0 & 0x0F == 1:
                    t = json.loads(payload)["time_ns"]
                    self.snapshots.append((now, t))
        except (EOFError, OSError):
            pass


def run(binary: str, targets: int, scans: int, window: float, period: float, seed: int) -> list[float]:
    with tempfile.TemporaryDirectory() as tmp:
        cfg = os.path.join(tmp, "sensors.cfg")
        with open(cfg, "w", encoding="ascii") as f:
            f.write("sifted-sensors 1\nsensor 1 range_bearing 0 0 0\n")
        udp, ws = free_port(socket.SOCK_DGRAM), free_port(socket.SOCK_STREAM)
        proc = subprocess.Popen(
            [binary, "--sensors", cfg, "--listen", f"127.0.0.1:{udp}", "--ws-port", str(ws),
             "--snapshot-interval", "0", "--reorder-window", str(window), "--stats-interval", "0",
             "--max-tracks", str(max(1024, 4 * targets)), "--queue", str(max(4096, 4 * targets))],
            stderr=subprocess.DEVNULL)
        try:
            reader = WsReader(ws)
            reader.start()
            rng = random.Random(seed)
            tg = []
            for _ in range(targets):
                rr, a, h, v = 2000 + 40000 * rng.random(), 2 * math.pi * rng.random(), \
                    2 * math.pi * rng.random(), 50 + 200 * rng.random()
                tg.append([rr * math.cos(a), rr * math.sin(a), v * math.cos(h), v * math.sin(h)])
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tx.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
            t0 = time.time_ns()
            sent: list[tuple[int, int]] = []  # (scan time_ns, monotonic ns of first send)
            seq = 0
            start = time.monotonic()
            for k in range(scans):
                time.sleep(max(0.0, start + k * period - time.monotonic()))
                t_ns = t0 + round(k * period * 1e9)
                datagrams = []
                for x in tg:
                    if k:
                        x[0] += x[2] * period
                        x[1] += x[3] * period
                    r = math.hypot(x[0], x[1]) + rng.gauss(0, 10.0)
                    b = math.remainder(math.atan2(x[1], x[0]) + rng.gauss(0, math.radians(0.2)),
                                       2 * math.pi)
                    datagrams.append(frame(seq, t_ns, max(r, 1.0), b))
                    seq += 1
                first = time.monotonic_ns()
                for d in datagrams:
                    tx.sendto(d, ("127.0.0.1", udp))
                sent.append((t_ns, first))
            time.sleep(1.0 + window)
            reader.stop = True
        finally:
            proc.terminate()
            proc.wait(timeout=10)
        lat = []
        for t_ns, first in sent:
            arrivals = [a for a, t in reader.snapshots if t == t_ns]
            arrival = max(arrivals) if arrivals else None
            if arrival is not None:
                lat.append((arrival - first) / 1e6)
        return lat


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("binary")
    ap.add_argument("--targets", default="10,100,1000")
    ap.add_argument("--scans", type=int, default=30)
    ap.add_argument("--windows", default="0,0.5")
    ap.add_argument("--period", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    print("bench,targets,reorder_window_s,scans_measured,median_ms,p95_ms,p99_ms,max_ms")
    for w in (float(x) for x in a.windows.split(",")):
        for n in (int(x) for x in a.targets.split(",")):
            lat = sorted(run(a.binary, n, a.scans, w, a.period, a.seed))
            if not lat:
                print(f"e2e,{n},{w},0,,,,")
                continue
            q = lambda p: lat[min(len(lat) - 1, round(p * (len(lat) - 1)))]  # noqa: E731
            print(f"e2e,{n},{w},{len(lat)},{statistics.median(lat):.2f},{q(0.95):.2f},"
                  f"{q(0.99):.2f},{lat[-1]:.2f}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
