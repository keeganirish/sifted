"""Deterministic degraded-network model and a live UDP impairment proxy.

Python mirror of src/sifted/net/impairment.hpp (normative description there
and in docs/adr/0008-impairment-model.md). For the same seed, configuration
and frames, impair() returns the same arrival sequence as the C++ impair();
tests/fixtures/impairment/ holds a golden output both sides check.

Offline:

    arrivals = impair([(send_ns, frame), ...], ImpairmentConfig(seed=7, ...))

Live proxy, between the sender and sifted_fusiond:

    python3 -m sifted_sim.impair --listen 127.0.0.1:5601 --target 127.0.0.1:5600 \
        --seed 7 --loss 0.02 --jitter-ms 300 --duplicate 0.01 --outage 30:40
    python3 -m sifted_sim.send crossing_pair --target 127.0.0.1:5601

The proxy timestamps each datagram on arrival (monotonic clock, relative to
the proxy's start, which is also the zero of --outage windows), asks the same
per-sensor Link what happens to it, and forwards each surviving copy at its
arrival time. Its wall-clock pacing is not deterministic (OS scheduling), but
which frames are lost or duplicated, and the delay each copy is given, are a
pure function of the seed, the configuration and each sensor's frame order.
Determinism of the run that matters -- the daemon's -- comes from its event
log, which records what actually arrived.
"""

from __future__ import annotations

import argparse
import heapq
import pathlib
import socket
import sys
import threading
import time
from dataclasses import dataclass, field

MASK64 = (1 << 64) - 1
MAX_DELAY_NS = 3_600_000_000_000


class SplitMix64:
    """SplitMix64 (Steele, Lea and Flood, 2014); same output as the C++ class."""

    def __init__(self, seed: int) -> None:
        self.state = seed & MASK64

    def next(self) -> int:
        self.state = (self.state + 0x9E3779B97F4A7C15) & MASK64
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & MASK64
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & MASK64
        return z ^ (z >> 31)

    def uniform(self) -> float:
        return (self.next() >> 11) * 2.0 ** -53

    def below(self, n: int) -> int:
        return (self.next() * n) >> 64


@dataclass(frozen=True)
class Outage:
    start_ns: int
    end_ns: int


@dataclass(frozen=True)
class LinkProfile:
    loss: float = 0.0
    p_good_to_bad: float = 0.0
    p_bad_to_good: float = 1.0
    loss_bad: float = 1.0
    delay_ns: int = 0
    jitter_ns: int = 0
    duplicate: float = 0.0
    outages: tuple[Outage, ...] = ()

    def validate(self) -> None:
        for name in ("loss", "p_good_to_bad", "p_bad_to_good", "loss_bad", "duplicate"):
            v = getattr(self, name)
            if not 0.0 <= v <= 1.0:
                raise ValueError(f"{name} must be a probability, got {v}")
        if not (0 <= self.delay_ns <= MAX_DELAY_NS and 0 <= self.jitter_ns <= MAX_DELAY_NS):
            raise ValueError("delay and jitter must be in [0, 1 h]")
        for o in self.outages:
            if not o.start_ns < o.end_ns:
                raise ValueError("an outage must be a non-empty [start, end) interval")


@dataclass(frozen=True)
class ImpairmentConfig:
    seed: int = 1
    global_profile: LinkProfile = LinkProfile()
    sensors: dict[int, LinkProfile] = field(default_factory=dict)
    outages: tuple[Outage, ...] = ()

    def validate(self) -> None:
        self.global_profile.validate()
        for p in self.sensors.values():
            p.validate()
        LinkProfile(outages=self.outages).validate()

    def profile(self, sensor: int) -> LinkProfile:
        return self.sensors.get(sensor, self.global_profile)


@dataclass
class Stats:
    sent: int = 0
    lost_outage: int = 0
    lost_good: int = 0
    lost_bad: int = 0
    duplicated: int = 0
    delivered: int = 0
    reordered: int = 0


def frame_sensor(frame: bytes) -> int:
    return frame[8] | (frame[9] << 8) if len(frame) >= 10 else 0


def link_seed(seed: int, sensor: int) -> int:
    return SplitMix64((seed ^ ((sensor * 0xD1B54A32D192ED03) & MASK64)) & MASK64).next()


class Link:
    """One sensor's link: feed frames in send order, get 0, 1 or 2 arrival times."""

    def __init__(self, seed: int, sensor: int, profile: LinkProfile,
                 shared_outages: tuple[Outage, ...] = ()) -> None:
        self.rng = SplitMix64(link_seed(seed, sensor))
        self.profile = profile
        self.outages = tuple(profile.outages) + tuple(shared_outages)
        self.bad = False

    def send(self, t_ns: int, stats: Stats | None = None) -> list[int]:
        p, rng = self.profile, self.rng
        # Always exactly five draws, in this order (as in C++).
        u_chain = rng.uniform()
        u_loss = rng.uniform()
        j1 = rng.below(p.jitter_ns + 1)
        u_dup = rng.uniform()
        j2 = rng.below(p.jitter_ns + 1)
        self.bad = (not u_chain < p.p_bad_to_good) if self.bad else u_chain < p.p_good_to_bad
        s = stats if stats is not None else Stats()
        s.sent += 1
        if any(o.start_ns <= t_ns < o.end_ns for o in self.outages):
            s.lost_outage += 1
            return []
        if u_loss < (p.loss_bad if self.bad else p.loss):
            if self.bad:
                s.lost_bad += 1
            else:
                s.lost_good += 1
            return []
        out = [t_ns + p.delay_ns + j1]
        if u_dup < p.duplicate:
            out.append(t_ns + p.delay_ns + j2)
            s.duplicated += 1
        s.delivered += len(out)
        return out


def impair(frames: list[tuple[int, bytes]], cfg: ImpairmentConfig,
           stats: Stats | None = None) -> list[tuple[int, int, bool, bytes]]:
    """(arrival_ns, source index, is_duplicate, frame), sorted like the C++ impair()."""
    cfg.validate()
    links: dict[int, Link] = {}
    s = stats if stats is not None else Stats()
    out = []
    for i, (t_ns, frame) in enumerate(frames):
        sensor = frame_sensor(frame)
        if sensor not in links:
            links[sensor] = Link(cfg.seed, sensor, cfg.profile(sensor), cfg.outages)
        for c, arrival in enumerate(links[sensor].send(t_ns, s)):
            out.append((arrival, i, c == 1, frame))
    out.sort(key=lambda a: (a[0], a[1], a[2]))
    latest = None
    for a in out:
        if latest is not None and a[1] < latest:
            s.reordered += 1
        latest = a[1] if latest is None else max(latest, a[1])
    return out


# ---- command line: profiles and the live proxy -------------------------------

def _outage(text: str) -> Outage:
    a, _, b = text.partition(":")
    return Outage(round(float(a) * 1e9), round(float(b) * 1e9))


def _profile_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--loss", type=float, default=0.0, help="per-frame loss probability (Good state)")
    ap.add_argument("--burst", type=float, nargs=3, metavar=("P_GB", "P_BG", "LOSS_BAD"),
                    help="Gilbert-Elliott burst loss: Good->Bad, Bad->Good, loss in Bad")
    ap.add_argument("--delay-ms", type=float, default=0.0, help="fixed one-way delay")
    ap.add_argument("--jitter-ms", type=float, default=0.0, help="+ uniform delay in [0, J]")
    ap.add_argument("--duplicate", type=float, default=0.0, help="duplication probability")


def _profile_from(ns: argparse.Namespace, outages: tuple[Outage, ...] = ()) -> LinkProfile:
    gb = ns.burst or (0.0, 1.0, 1.0)
    return LinkProfile(loss=ns.loss, p_good_to_bad=gb[0], p_bad_to_good=gb[1], loss_bad=gb[2],
                       delay_ns=round(ns.delay_ms * 1e6), jitter_ns=round(ns.jitter_ms * 1e6),
                       duplicate=ns.duplicate, outages=outages)


def parse_sensor_override(text: str) -> tuple[int, LinkProfile]:
    """'ID:key=value,...' with keys loss, burst=P_GB/P_BG/LOSS_BAD, delay_ms, jitter_ms,
    duplicate, outage=START/END (seconds; repeatable)."""
    sid, _, rest = text.partition(":")
    kw: dict = {}
    outages = []
    for item in filter(None, rest.split(",")):
        k, _, v = item.partition("=")
        if k == "loss":
            kw["loss"] = float(v)
        elif k == "burst":
            a, b, c = (float(x) for x in v.split("/"))
            kw.update(p_good_to_bad=a, p_bad_to_good=b, loss_bad=c)
        elif k == "delay_ms":
            kw["delay_ns"] = round(float(v) * 1e6)
        elif k == "jitter_ms":
            kw["jitter_ns"] = round(float(v) * 1e6)
        elif k == "duplicate":
            kw["duplicate"] = float(v)
        elif k == "outage":
            outages.append(_outage(v.replace("/", ":")))
        else:
            raise ValueError(f"unknown sensor override key {k!r}")
    return int(sid), LinkProfile(outages=tuple(outages), **kw)


def config_from_args(ns: argparse.Namespace) -> ImpairmentConfig:
    cfg = ImpairmentConfig(seed=ns.seed, global_profile=_profile_from(ns),
                           sensors=dict(parse_sensor_override(s) for s in ns.sensor),
                           outages=tuple(_outage(o) for o in ns.outage))
    cfg.validate()
    return cfg


class Proxy:
    """Receives datagrams, applies the model, forwards each copy at its arrival time."""

    def __init__(self, cfg: ImpairmentConfig, listen: tuple[str, int], target: tuple[str, int],
                 max_held: int = 65536) -> None:
        self.cfg = cfg
        self.target = target
        self.max_held = max_held
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(listen)
        self.rx.settimeout(0.1)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.links: dict[int, Link] = {}
        self.stats = Stats()
        self.overflow = 0
        self.heap: list[tuple[int, int, bytes]] = []
        self.order = 0
        self.cv = threading.Condition()
        self.stop = threading.Event()
        self.t0 = time.monotonic_ns()

    def now_ns(self) -> int:
        return time.monotonic_ns() - self.t0

    def _forwarder(self) -> None:
        while not self.stop.is_set() or self.heap:
            with self.cv:
                if not self.heap:
                    if self.stop.is_set():
                        return
                    self.cv.wait(0.1)
                    continue
                due, _, frame = self.heap[0]
                wait = (due - self.now_ns()) / 1e9
                if wait > 0:
                    self.cv.wait(min(wait, 0.1))
                    continue
                heapq.heappop(self.heap)
            self.tx.sendto(frame, self.target)

    def run(self, duration_s: float | None = None) -> None:
        fwd = threading.Thread(target=self._forwarder, daemon=True)
        fwd.start()
        end = None if duration_s is None else self.now_ns() + round(duration_s * 1e9)
        try:
            while end is None or self.now_ns() < end:
                try:
                    frame, _ = self.rx.recvfrom(65536)
                except socket.timeout:
                    continue
                t = self.now_ns()
                sensor = frame_sensor(frame)
                if sensor not in self.links:
                    self.links[sensor] = Link(self.cfg.seed, sensor, self.cfg.profile(sensor),
                                              self.cfg.outages)
                with self.cv:
                    for arrival in self.links[sensor].send(t, self.stats):
                        if len(self.heap) >= self.max_held:
                            self.overflow += 1  # bounded: never hold more than max_held
                            continue
                        heapq.heappush(self.heap, (arrival, self.order, frame))
                        self.order += 1
                    self.cv.notify()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()
            fwd.join(timeout=5.0)
            self.rx.close()
            self.tx.close()
            s = self.stats
            print(f"impair: sent {s.sent}, lost {s.lost_good + s.lost_bad} (burst {s.lost_bad}), "
                  f"outage {s.lost_outage}, duplicated {s.duplicated}, delivered {s.delivered}, "
                  f"overflow {self.overflow}", file=sys.stderr)


# The configuration of the cross-language golden fixture
# (tests/fixtures/impairment/; checked by sim/tests/test_impair.py and
# tests/impairment_test.cpp, which builds the same configuration in C++).
GOLDEN_SCENARIO = "crossing_pair"
GOLDEN_CONFIG = ImpairmentConfig(
    seed=20261115,
    global_profile=LinkProfile(loss=0.05, p_good_to_bad=0.02, p_bad_to_good=0.25, loss_bad=0.7,
                               delay_ns=30_000_000, jitter_ns=400_000_000, duplicate=0.03),
    sensors={20: LinkProfile(loss=0.1, jitter_ns=50_000_000,
                             outages=(Outage(60_000_000_000, 62_500_000_000),))},
    outages=(Outage(30_000_000_000, 35_000_000_000),),
)


def golden_lines(frames: list[bytes]) -> list[str]:
    """Each frame is sent at its measurement time; one line per arrival."""
    from .send import frame_time_ns
    stats = Stats()
    arrivals = impair([(frame_time_ns(f), f) for f in frames], GOLDEN_CONFIG, stats)
    lines = [f"# stats sent={stats.sent} lost_outage={stats.lost_outage} "
             f"lost_good={stats.lost_good} lost_bad={stats.lost_bad} "
             f"duplicated={stats.duplicated} delivered={stats.delivered} "
             f"reordered={stats.reordered}"]
    lines += [f"{a} {i} {int(d)}" for a, i, d, _ in arrivals]
    return lines


def _addr(text: str) -> tuple[str, int]:
    host, _, port = text.rpartition(":")
    return host, int(port)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted_sim.impair", description=__doc__.splitlines()[0])
    ap.add_argument("--listen", default="127.0.0.1:5601", help="proxy UDP address [127.0.0.1:5601]")
    ap.add_argument("--target", default="127.0.0.1:5600", help="daemon UDP address [127.0.0.1:5600]")
    ap.add_argument("--seed", type=int, default=1)
    _profile_args(ap)
    ap.add_argument("--outage", action="append", default=[], metavar="START:END",
                    help="seconds after proxy start with no delivery on any link (repeatable)")
    ap.add_argument("--sensor", action="append", default=[], metavar="ID:key=value,...",
                    help="per-sensor profile replacing the global one (repeatable); keys: loss, "
                         "burst=a/b/c, delay_ms, jitter_ms, duplicate, outage=s/e")
    ap.add_argument("--duration", type=float, default=None, help="stop after S seconds")
    ap.add_argument("--write-golden", metavar="FILE", help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    if args.write_golden:
        from .send import load_frames
        text = "\n".join(golden_lines(load_frames(GOLDEN_SCENARIO))) + "\n"
        pathlib.Path(args.write_golden).write_text(text, encoding="ascii", newline="\n")
        return 0
    try:
        cfg = config_from_args(args)
    except ValueError as e:
        ap.error(str(e))
    print(f"impair: {args.listen} -> {args.target}, seed {cfg.seed}", file=sys.stderr)
    Proxy(cfg, _addr(args.listen), _addr(args.target)).run(args.duration)
    return 0


if __name__ == "__main__":
    sys.exit(main())
