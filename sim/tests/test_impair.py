import pathlib
import socket
import threading
import time
import unittest

from sifted_sim import faults, impair, send, wire

ROOT = pathlib.Path(__file__).resolve().parents[2]
GOLDEN = ROOT / "tests" / "fixtures" / "impairment" / "golden.txt"
FIXTURE = ROOT / "tests" / "fixtures" / "scenarios" / impair.GOLDEN_SCENARIO


def frame(sensor: int, seq: int, t_ns: int) -> bytes:
    return wire.encode(wire.Measurement(sensor, wire.RANGE_BEARING, seq, t_ns,
                                        (4000.0, 0.5, 0.0), (10.0, 0.0035, 0.0)))


class ImpairTest(unittest.TestCase):
    def test_splitmix_reference_values(self) -> None:
        r = impair.SplitMix64(0)
        self.assertEqual([r.next() for _ in range(3)],
                         [0xE220A8397B1DCDAF, 0x6E789E6AA1B965F4, 0x06C45D188009454F])

    def test_golden_fixture_matches(self) -> None:
        # Same fixture the C++ Impairment.MatchesPythonGoldenFixture test reads,
        # computed from the committed scenario frames.
        frames = send.load_frames(str(FIXTURE))
        self.assertEqual(impair.golden_lines(frames),
                         GOLDEN.read_text(encoding="ascii").splitlines())

    def test_identity_without_impairment(self) -> None:
        frames = [(i * 10_000_000, frame(1, i, i * 10_000_000)) for i in range(50)]
        out = impair.impair(frames, impair.ImpairmentConfig())
        self.assertEqual([(a, i, d) for a, i, d, _ in out], [(t, i, False) for i, (t, _) in
                                                             enumerate(frames)])

    def test_loss_rate_and_validation(self) -> None:
        frames = [(i, frame(1, i, i)) for i in range(20000)]
        s = impair.Stats()
        impair.impair(frames, impair.ImpairmentConfig(global_profile=impair.LinkProfile(loss=0.2)), s)
        self.assertAlmostEqual(s.lost_good / s.sent, 0.2, delta=0.015)
        with self.assertRaises(ValueError):
            impair.ImpairmentConfig(global_profile=impair.LinkProfile(loss=2.0)).validate()

    def test_sensor_override_parsing(self) -> None:
        sid, p = impair.parse_sensor_override("20:loss=0.1,burst=0.01/0.3/0.9,jitter_ms=50,outage=3/4")
        self.assertEqual(sid, 20)
        self.assertEqual((p.loss, p.p_good_to_bad, p.p_bad_to_good, p.loss_bad), (0.1, 0.01, 0.3, 0.9))
        self.assertEqual(p.jitter_ns, 50_000_000)
        self.assertEqual(p.outages, (impair.Outage(3_000_000_000, 4_000_000_000),))

    def test_live_proxy_forwards_and_drops(self) -> None:
        # Real sockets on loopback: an outage covering the whole run drops
        # everything; without one, every datagram is forwarded.
        for outage, expect_all in ((None, True), ((0.0, 3600.0), False)):
            target = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            target.bind(("127.0.0.1", 0))
            target.settimeout(0.5)
            listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            listen.bind(("127.0.0.1", 0))
            listen_addr = listen.getsockname()
            listen.close()
            outages = (impair.Outage(round(outage[0] * 1e9), round(outage[1] * 1e9)),) if outage else ()
            cfg = impair.ImpairmentConfig(seed=3, global_profile=impair.LinkProfile(jitter_ns=5_000_000),
                                          outages=outages)
            proxy = impair.Proxy(cfg, listen_addr, target.getsockname())
            t = threading.Thread(target=proxy.run, args=(1.5,))
            t.start()
            time.sleep(0.1)
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sent = [frame(1, i, i) for i in range(20)]
            for f in sent:
                tx.sendto(f, listen_addr)
            got = []
            try:
                while len(got) < len(sent):
                    got.append(target.recvfrom(256)[0])
            except socket.timeout:
                pass
            t.join()
            tx.close()
            target.close()
            if expect_all:
                self.assertEqual(sorted(got), sorted(sent))
            else:
                self.assertEqual(got, [])
                self.assertEqual(proxy.stats.lost_outage, len(sent))


class FaultsTest(unittest.TestCase):
    def items(self) -> list[faults.Sent]:
        out = []
        for k in range(10):  # 10 scans of 2 detections from sensor 1, and sensor 2 at 2 Hz
            t = k * 1_000_000_000
            out += [faults.Sent(t, send.decode(frame(1, 2 * k, t))),
                    faults.Sent(t, send.decode(frame(1, 2 * k + 1, t))),
                    faults.Sent(t + 500_000_000, send.decode(frame(2, k, t + 500_000_000)))]
        return out

    def test_restart_resets_seq_only(self) -> None:
        out = faults.apply("restart:1@5", self.items())
        s1 = [i.m.seq for i in out if i.m.sensor_id == 1]
        self.assertEqual(s1[:10], list(range(10)))
        self.assertEqual(s1[10:], list(range(10)))
        self.assertEqual([i.m.time_ns for i in out], [i.m.time_ns for i in self.items()])

    def test_clock_offset_keeps_send_time(self) -> None:
        out = faults.apply("clock:2:+2", self.items())
        for a, b in zip(out, self.items()):
            self.assertEqual(a.send_ns, b.send_ns)
            self.assertEqual(a.m.time_ns - b.m.time_ns, 2_000_000_000 if a.m.sensor_id == 2 else 0)

    def test_dropout_stuck_bias_intermittent(self) -> None:
        self.assertFalse(any(i.m.sensor_id == 1 and 3e9 <= i.m.time_ns < 6e9
                             for i in faults.apply("dropout:1@3-6", self.items())))
        stuck = [i for i in faults.apply("stuck:1@4-7", self.items()) if i.m.sensor_id == 1]
        frozen = [i.m.z for i in stuck if 4e9 <= i.m.time_ns < 7e9]
        self.assertEqual(len(frozen), 6)
        self.assertEqual(len(set(frozen)), 1)  # the identical scan, repeated
        biased = faults.apply("bias:1:50", self.items())
        self.assertEqual(biased[0].m.z[0], 4050.0)
        a = faults.apply("intermittent:2:0.5", self.items(), seed=4)
        self.assertEqual(a, faults.apply("intermittent:2:0.5", self.items(), seed=4))
        self.assertLess(len(a), len(self.items()))
        with self.assertRaises(ValueError):
            faults.apply("melt:1", self.items())

    def test_sender_schedules_by_true_send_time(self) -> None:
        frames = [frame(1, 0, 0), frame(2, 0, 500_000_000)]
        out, sends = send.with_faults(frames, ["clock:2:+3"], 1)
        plan = send.schedule(out, 1.0, None, sends)
        self.assertEqual([round(o, 3) for o, _ in plan], [0.0, 0.5])
        self.assertEqual(send.frame_time_ns(plan[1][1]), 3_500_000_000)


if __name__ == "__main__":
    unittest.main()
