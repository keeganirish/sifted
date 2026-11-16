# Sifted

**Signal, sifted.** Deterministic multi-sensor track fusion in C++.

Sifted takes noisy, partial, and sometimes conflicting measurements from many sensors and turns them into one coherent, trustworthy picture of what is out there. Simulated sensors and a live public ADS-B feed publish measurements to a central fusion service, which estimates each object's position and velocity, decides which measurements belong to which object, handles objects appearing and disappearing, and streams the resulting tracks to a live map.

It is a small-scale, from-scratch take on the core problem behind modern situational-awareness software. The goal is a working, well-tested, well-documented system that demonstrates systems-level engineering: determinism, traceability, robust handling of untrusted input, thread safety, networking, and measured performance.

> **Status:** working end to end: multi-sensor tracking, record and replay, the live daemon and map, degraded-network handling, fault injection, fuzzing and benchmarks. All C++ and Python tests pass under the dev, release, ASan/UBSan and TSan presets. The OpenSky adapter is tested offline (sample responses and property tests) but has not yet been run against the live API. Design and decisions: [`docs/design.md`](docs/design.md).

## Features

- **Sensor simulation.** *(built)* Radar-like (range and bearing) and camera-like (bearing-only) sensors generate noisy measurements of moving objects, each with its own noise, update rate, coverage, and failure modes (missed detections, clutter, dropouts).
- **Real data feed.** *(built; tested offline)* Public ADS-B aircraft data from the OpenSky Network is ingested as an additional sensor through a small adapter, so the system can run on live, messy data and not only on simulation.
- **Track estimation.** *(built)* Kalman-family filters (Kalman and extended Kalman) estimate each object's state and quantify the uncertainty of that estimate, with manoeuvre detection.
- **Data association.** *(built)* Each measurement is matched to an existing track or starts a new one, using Mahalanobis gating and global nearest-neighbour assignment. Tracks that stop receiving measurements are coasted, then dropped.
- **Operator view.** *(built)* A web client subscribes to the track stream and shows a live map with track identity, confidence, and contributing sensors.
- **Deterministic record and replay.** *(built)* Any run can be recorded and replayed so that identical input produces bit-for-bit identical output. Time is an input to the system, not read from the machine.
- **Traceability.** *(built)* Every track can be traced back to the measurements and decisions that produced it, so you can ask why the system believes something.
- **Degraded-network behavior.** *(built)* Packet loss (independent and bursty), delay, jitter-induced reordering, duplication and link outages are simulated with one seeded model, offline in tests and live as a UDP proxy; a bounded reorder stage keeps jitter from becoming data loss; recovery after outages is measured ([design §9](docs/design.md#9-degraded-network-behaviour)).
- **Fuzzing and fault injection.** *(built)* Every parser of untrusted input (wire frames, event logs and their headers, sensor tables, the WebSocket handshake and client frames) and the engine itself are fuzzed with libFuzzer; the OpenSky adapter is property-tested. Sensor restarts, dropouts, clock faults, bias, stuck sensors, clutter floods, duplicate floods and malformed bursts are injected and their effects tested ([threat model](docs/threat-model.md#sensor-and-input-faults)).
- **Measured performance.** *(built)* Per-scan latency distributions, throughput, tracks per core and end-to-end daemon latency ([benchmarks](docs/benchmarks.md)): about 3 000 targets per engine thread within a 100 ms p99 budget at 1 Hz on a 2-vCPU VM.

## Architecture

```
 Simulated radar   --\
 Simulated camera  ---+--UDP-->  ingest + validate --> associate --> estimate --> track store --WebSocket--> operator map
 ADS-B (OpenSky)   --/                  |                                                |
                                        v                                                v
                                   event log  <------------- replay (simulated clock) ---+
```

Every message crosses a validated boundary: explicit versioning, size limits, and strict parsing on everything received. The engine itself is single-threaded and deterministic; network receive and WebSocket publication run on their own threads with documented ownership of shared state, checked with ThreadSanitizer.

## Tech stack

| Area | Choice |
|---|---|
| Core | C++17/20, CMake |
| Math | Eigen (Kalman filters), nearest-neighbor gating with optional global nearest neighbor (Hungarian) |
| Networking | UDP for sensor data, a minimal read-only WebSocket server for operator clients, POSIX sockets (ADR 0006) |
| Message format | A small custom framing layer, versioned and size-limited (ADR 0001) |
| Record and replay | Append-only timestamped event log, simulated clock |
| Simulators and tooling | Python (sensor simulators, analysis, benchmark plots) |
| Operator display | HTML and JavaScript, Leaflet with OpenStreetMap tiles |
| Testing | GoogleTest or Catch2, ground-truth scenarios as a regression benchmark |
| Quality | libFuzzer or AFL++, ASan, UBSan, TSan, clang-tidy |
| CI | GitHub Actions |
| Optional | CUDA-accelerated association, benchmarked against the CPU path |

## Getting started

Requires CMake 3.21+, Ninja, a C++20 compiler (GCC 13 or Clang 18 are tested), Eigen 3.4 and GoogleTest.

```bash
sudo apt install cmake ninja-build libeigen3-dev libgtest-dev   # Debian/Ubuntu

cmake --preset dev            # configure (Debug, warnings as errors)
cmake --build --preset dev
ctest --preset dev
```

Other presets: `release`, `asan` (ASan + UBSan), `tsan`, and `fuzz` (Clang + libFuzzer).
No system packages? Add `-DSIFTED_FETCH_DEPS=ON` to download pinned copies (untested so far).

**ThreadSanitizer on WSL2 and other recent kernels:** if every TSan binary
fails with "unexpected memory mapping", the kernel randomises mmap with more
bits than TSan supports. This is environmental, not a code bug. Run
`sudo sysctl vm.mmap_rnd_bits=28` once (until reboot), or run the tests
under `setarch "$(uname -m)" -R ctest --preset tsan`. CI does the former.

```bash
cmake --preset fuzz && cmake --build --preset fuzz     # needs libclang-rt-18-dev
mkdir -p build/fuzz-work/engine
build/fuzz/fuzz/fuzz_engine build/fuzz-work/engine fuzz/corpus_engine -max_total_time=60
```

Six targets (`fuzz_decode`, `fuzz_event_log`, `fuzz_log_header`,
`fuzz_sensor_table`, `fuzz_websocket`, `fuzz_engine`); see
[`fuzz/README.md`](fuzz/README.md).

### Run the daemon with simulated sensors

```bash
# 1. Write a scenario's sensor table (and frames) somewhere.
cd sim && python3 -m sifted_sim.generate crossing_pair /tmp/crossing && cd ..

# 2. Start the fusion service: UDP in on 5600, WebSocket out on 8765, recording a log.
build/dev/src/apps/sifted_fusiond --sensors /tmp/crossing/sensors.cfg \
    --record /tmp/crossing.siftlog --output /tmp/live.txt

# 3. Serve the map and open http://localhost:8000/ (it connects to ws://localhost:8765/;
#    use ?ws=ws://host:port/ for another address). Not file://: OpenStreetMap blocks
#    tile requests without a Referer, so every tile shows "403 Access blocked".
(cd web && python3 -m http.server 8000) &

# 4. In another terminal, send the scenario at 5x real time.
cd sim && python3 -m sifted_sim.send crossing_pair --speed 5

# 5. Ctrl-C the daemon, then replay the recording and compare outputs.
build/dev/src/apps/sifted_fusiond --replay /tmp/crossing.siftlog --output /tmp/replay.txt --ws-port 0
cmp /tmp/live.txt /tmp/replay.txt && echo identical
# Or replay at real time onto the map:  --replay /tmp/crossing.siftlog --pace 1 --hold
```

Simulated scenarios use a local x/y frame; for display (and for geodetic
input) the daemon maps it to latitude/longitude through a tangent-plane origin,
`--origin lat,lon[,alt]` (default 51.4779,-0.0015, near Greenwich). Run
`sifted_fusiond --help` for every option.

### Degraded networks and faults

```bash
# Impairment proxy between sender and daemon (ADR 0008): loss with bursts,
# jitter (and so reordering), duplicates, an outage 30-38 s after it starts.
cd sim && python3 -m sifted_sim.impair --listen 127.0.0.1:5601 --target 127.0.0.1:5600 \
    --seed 7 --loss 0.03 --burst 0.01 0.25 0.6 --jitter-ms 400 --duplicate 0.02 --outage 30:38
# ... and send through it, optionally with a sensor fault (sim/sifted_sim/faults.py):
python3 -m sifted_sim.send crossing_pair --target 127.0.0.1:5601 --fault restart:10@30
```

The daemon holds frames for `--reorder-window` (default 0.5 s) and releases
them in measurement-time order (ADR 0009); `--max-time-ahead S` rejects frames
from sensors whose clocks run ahead (ADR 0010; needs wall-clock senders, e.g.
`send.py --rebase-now`); `--rate-limit` caps datagrams per source address.
The demo runbook is [`docs/demo.md`](docs/demo.md) (`scripts/demo.sh` runs it).

### Benchmarks

```bash
cmake --preset release && cmake --build --preset release
build/release/benchmarks/sifted_bench all --runs 5 > engine.csv     # ~15 min
python3 benchmarks/e2e_latency.py build/release/src/apps/sifted_fusiond > e2e.csv
python3 benchmarks/report.py <dir with the CSVs>                     # tables (+ plot)
```

Results and method: [`docs/benchmarks.md`](docs/benchmarks.md).

### ADS-B (OpenSky)

```bash
cd sim && python3 -m sifted_sim.adsb --write-sensors /tmp/adsb.cfg --sensor-id 100
build/dev/src/apps/sifted_fusiond --sensors /tmp/adsb.cfg --origin 51.47,-0.45 \
    --initial-velocity-sigma 250 --tentative-timeout 90 --coast-after 60 --drop-after 180 \
    --max-position-sigma 10000
export OPENSKY_CLIENT_ID=... OPENSKY_CLIENT_SECRET=...   # optional; anonymous quota is tiny
cd sim && python3 -m sifted_sim.adsb --bbox 51.0,52.0,-1.2,0.4 --sensor-id 100
```

The adapter keeps to OpenSky's documented daily credit quota: for a small
bounding box that means one poll every ~22 s when authenticated and every
~216 s anonymously, so the lifecycle options above are needed (the defaults
assume ~1 Hz sensors). Choose an origin inside the box (ADR 0007). Use of
OpenSky data is subject to the OpenSky Network's terms of use.

## Layout

```
src/sifted/   library: core (types, clock, sensor table, outcomes, multi-track engine),
              wire (codec), estimation (Kalman filter, EKF models, geodetic, gate screen),
              association (gating, assignment, lifecycle), store (track table,
              decision trace), replay (event log, replayer), net (ingest pipeline with
              dedup, clock gate and reorder stage, snapshots, impairment model, UDP,
              rate limiting, queue, WebSocket)
src/apps/     sifted_fusiond, the fusion daemon (live and replay)
sim/          sensor simulators, scenarios, UDP sender with fault injection,
              impairment proxy, OpenSky adapter (Python)
web/          operator map client (Leaflet, no build step)
tests/        unit tests, fixtures, ground-truth regression, degraded-network,
              fault-injection and UDP flood tests
fuzz/         six libFuzzer targets and seed corpora
benchmarks/   engine benchmark harness, end-to-end latency script, report generator
scripts/      demo launcher
docs/         design document, threat model, ADRs 0001-0011, benchmark report, demo runbook
cmake/        build options, sanitizers, dependencies
```

## Milestones

- **M0:** project skeleton, CI, sanitizers, fuzz harness, wire format, simulated clock.
- **M1:** radar simulator, sensor table, single-sensor Kalman/EKF tracker, ground-truth regression tests.
- **M2:** multi-sensor association and track lifecycle, camera simulator, decision trace, record and replay, UDP ingest and daemon, WebSocket stream and map, ADS-B ingest.
- **M3:** degraded-network simulator and recovery tests, bounded reorder stage, fault injection, clock gate and per-source rate limiting, fuzzing of every parser and the engine, benchmarks, design document, threat model, demo runbook ([`docs/demo.md`](docs/demo.md)).
- **Possible next steps:** CUDA-accelerated association; a spatial index for more than ~3,000 tracks per engine thread; sensor bias estimation.

## Scope and ground rules

- Uses only public data and the project's own simulations. Nothing is based on controlled or non-public technical information.
- Focused on perception, tracking, and situational awareness. Nothing in the project involves weapons or targeting.
- Scope is cut before polish. A smaller system that works, is tested, and is clearly documented beats a large one that is half-finished.

## Data sources

Live aircraft data comes from the [OpenSky Network](https://opensky-network.org/). Use of that data is subject to the OpenSky Network's terms of use.

## License

Released under the [MIT License](LICENSE).
