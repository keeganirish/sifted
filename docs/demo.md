# Demo runbook

A step-by-step script for recording the M3 demo video: crossing targets, a
target entering and one leaving coverage, a manoeuvring target, camera
fusion, and a network outage with recovery, then a bit-for-bit replay.

Everything runs on one machine. You need four terminals (or use
`scripts/demo.sh`, which starts the pieces in order and leaves the map and
the narration to you) and a browser.

> The app does the same with one click: start Sifted (`./sifted`), choose
> **Guided demo**, network **Lossy + outage**, and press **Run scenario**; the
> side panel narrates the table in section 5 as it happens. This runbook is the
> manual version, with every process in its own terminal.

## 0. Build once

```bash
cmake --preset release && cmake --build --preset release
export FUSIOND=$PWD/build/release/src/apps/sifted_fusiond
```

The scenario is `demo` (`sim/sifted_sim/scenarios.py`; committed fixture in
`tests/fixtures/scenarios/demo/`). It runs 150 s of scenario time: five
targets, one radar (8 km coverage, ring on the map is not drawn -- mention
it), one camera at the north-west.

## 1. Start the daemon (terminal 1)

```bash
$FUSIOND --sensors tests/fixtures/scenarios/demo/sensors.cfg \
    --listen 127.0.0.1:5600 --record /tmp/demo.siftlog --output /tmp/demo_live.txt \
    --stats-interval 5
```

It prints `track snapshots on ws://127.0.0.1:8765/`. Defaults in effect:
0.5 s reorder window (ADR 0009), per-source rate limit (ADR 0010).

## 2. Open the map (browser)

Serve the map and open <http://localhost:8000/>:

```bash
(cd web && python3 -m http.server 8000) &
```

Not `file://`: OpenStreetMap refuses tile requests without a Referer, so every
tile would show "403 Access blocked". The page connects to
`ws://localhost:8765/`. Nothing is shown until tracks exist.

## 3. Start the impairment proxy (terminal 2)

```bash
cd sim
python3 -m sifted_sim.impair --listen 127.0.0.1:5601 --target 127.0.0.1:5600 \
    --seed 7 --loss 0.03 --burst 0.01 0.25 0.6 --delay-ms 50 --jitter-ms 150 \
    --duplicate 0.02 --outage 70:78
```

The outage is 70-78 s **after the proxy starts**, so start the sender (next
step) within a second or two of the proxy, at `--speed 1`, and the outage
lands at scenario time ~70 s, just after the crossing.

## 4. Send the scenario (terminal 3)

```bash
cd sim
python3 -m sifted_sim.send demo --target 127.0.0.1:5601 --speed 1
```

## 5. What to point out, in order

| Scenario time | On screen | Say |
|---|---|---|
| 0-5 s | tracks appear a few seconds after start | a track is shown once it is confirmed (3 radar hits); before that it is tentative and hidden |
| 5-60 s | two tracks converging in the middle | the crossing pair: 30° apart, inside each other's gates for ~10 s. Global nearest neighbour keeps the ids straight |
| ~20 s | a new track in the east | target 3 entering radar coverage from outside |
| any time | contributing sensors in the track table and tooltip | radar *and* camera: bearings from the camera's different viewpoint tighten the estimate (~25% lower error in the tests) |
| 50-80 s | the western track turns | manoeuvre detection raises process noise during the turn (the tooltip says "manoeuvring") |
| 70-78 s | all tracks stop updating, then turn "coasting" | the proxy has cut the link. Tracks coast on prediction; nothing is invented |
| ~78-82 s | tracks update again | recovery: straight-flying targets keep their ids; one that turned during the gap may come back under a new id. Typical recovery 1-4 s |
| ~95 s | the south-western track coasts and disappears | target 4 leaving coverage: coasting after 2.5 s, dropped after 10 s |
| terminal 1 | stats lines | duplicates suppressed, "reorder held", out-of-order 0: the jittered frames were put back in order |

Optional extras while it runs:
- a fault: in terminal 3 use `--fault clock:24:+2` (camera clock 2 s fast).
  Without a clock gate it stalls the radar's data (out-of-order count climbs).
  To show the mitigation, restart the daemon with `--max-time-ahead 0.5` and
  the sender with `--rebase-now` (the gate needs wall-clock timestamps).
- a flood: `python3 -c "import socket;s=socket.socket(2,2);[s.sendto(b'x'*40,('127.0.0.1',5600)) for _ in range(100000)]"`
  and watch `rate-limited` / `rejected` rise in terminal 1 while the tracks
  keep updating.

## 6. Stop and replay

When the sender prints `sent 1950 frames`, wait a few seconds, then Ctrl-C
the proxy and the daemon (terminal 1 prints final counters). Then:

```bash
$FUSIOND --replay /tmp/demo.siftlog --output /tmp/demo_replay.txt --ws-port 0
cmp /tmp/demo_live.txt /tmp/demo_replay.txt && echo identical
```

`identical` is the point: the live run had real packet loss, jitter and an
outage, and the recording reproduces it bit for bit. To show the replay on
the map at real time instead:

```bash
$FUSIOND --replay /tmp/demo.siftlog --pace 1 --hold
```

## Notes

- `--speed N` scales the scenario but not the proxy: jitter and the outage
  are in wall-clock time, so at `--speed 10` a 150 ms jitter is 1.5 s of
  scenario time. Use `--speed 1` for the video.
- Rehearsed in the sandbox (no browser there): daemon + proxy + sender at
  `--speed 10` with a sensor restart injected; 117 frames lost to the outage,
  89 to loss, 28 duplicates suppressed, 0 out of order, and the replay was
  byte-identical to the live output.
