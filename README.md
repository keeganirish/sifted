# Sifted

**Signal, sifted.** Deterministic multi-sensor track fusion in C++.

Sifted takes noisy, partial, and sometimes conflicting measurements from many sensors and turns them into one coherent, trustworthy picture of what is out there. Simulated sensors and a live public ADS-B feed publish measurements to a central fusion service, which estimates each object's position and velocity, decides which measurements belong to which object, handles objects appearing and disappearing, and streams the resulting tracks to a live map.

It is a small-scale, from-scratch take on the core problem behind modern situational-awareness software. The goal is a working, well-tested, well-documented system that demonstrates systems-level engineering: determinism, traceability, robust handling of untrusted input, thread safety, networking, and measured performance.

> **Status: milestones 0 and 1 complete; milestone 2 implemented, final verification pending.** Built: multi-target, multi-sensor tracking (radar + camera + ADS-B) with chi-square gating, global nearest-neighbour assignment, track lifecycle and a per-track decision trace; deterministic record and replay; the `sifted_fusiond` daemon (UDP in, WebSocket out); a Leaflet map; and an OpenSky ADS-B adapter. M2 is not marked complete until its tests have run against the real Eigen and GoogleTest packages, the libFuzzer targets have been built, and the map and live OpenSky feed have been tried for real (see the [roadmap](#roadmap)). Features below are marked as built only when they are; see [`docs/design.md`](docs/design.md).

## Features

- **Sensor simulation.** *(built)* Radar-like (range and bearing) and camera-like (bearing-only) sensors generate noisy measurements of moving objects, each with its own noise, update rate, coverage, and failure modes (missed detections, clutter, dropouts).
- **Real data feed.** *(built; tested offline)* Public ADS-B aircraft data from the OpenSky Network is ingested as an additional sensor through a small adapter, so the system can run on live, messy data and not only on simulation.
- **Track estimation.** *(built)* Kalman-family filters (Kalman and extended Kalman) estimate each object's state and quantify the uncertainty of that estimate, with manoeuvre detection.
- **Data association.** *(built)* Each measurement is matched to an existing track or starts a new one, using Mahalanobis gating and global nearest-neighbour assignment. Tracks that stop receiving measurements are coasted, then dropped.
- **Operator view.** *(built)* A web client subscribes to the track stream and shows a live map with track identity, confidence, and contributing sensors.
- **Deterministic record and replay.** *(built)* Any run can be recorded and replayed so that identical input produces bit-for-bit identical output. Time is an input to the system, not read from the machine.
- **Traceability.** *(built)* Every track can be traced back to the measurements and decisions that produced it, so you can ask why the system believes something.
- **Degraded-network behavior.** *(planned, M3)* Packet loss, delay, reordering, and dropped links can be simulated, with measurements of how quickly and how well the picture recovers.
- **Fuzzing and fault injection.** *(partly built)* Malformed packets and event logs are fuzzed; sensor dropouts and duplicate messages are part of the simulator and tests. Broader fault injection is M3.
- **Measured performance.** *(planned, M3)* Latency, throughput, and tracks handled per core are benchmarked and reported.

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

```bash
cmake --preset fuzz && cmake --build --preset fuzz     # needs libclang-rt-18-dev
mkdir -p build/fuzz-work build/fuzz-work-log
build/fuzz/fuzz/fuzz_decode build/fuzz-work fuzz/corpus -max_total_time=60
build/fuzz/fuzz/fuzz_event_log build/fuzz-work-log fuzz/corpus_log -max_total_time=60
```

### Run the daemon with simulated sensors

```bash
# 1. Write a scenario's sensor table (and frames) somewhere.
cd sim && python3 -m sifted_sim.generate crossing_pair /tmp/crossing && cd ..

# 2. Start the fusion service: UDP in on 5600, WebSocket out on 8765, recording a log.
build/dev/src/apps/sifted_fusiond --sensors /tmp/crossing/sensors.cfg \
    --record /tmp/crossing.siftlog --output /tmp/live.txt

# 3. Open web/index.html in a browser (it connects to ws://localhost:8765/;
#    use index.html?ws=ws://host:port/ for another address).

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
              wire (codec), estimation (Kalman filter, EKF models, geodetic),
              association (gating, assignment, lifecycle), store (track table,
              decision trace), replay (event log, replayer), net (ingest pipeline,
              dedup, snapshots, UDP, queue, WebSocket)
src/apps/     sifted_fusiond, the fusion daemon (live and replay)
sim/          sensor simulators, scenarios, UDP sender, OpenSky adapter (Python)
web/          operator map client (Leaflet, no build step)
tests/        unit tests, fixtures, ground-truth regression tests (single and multi-target)
fuzz/         libFuzzer targets (wire decoder, event log) and seed corpora
docs/         design document, threat model, ADRs 0001-0007; benchmark report to come
cmake/        build options, sanitizers, dependencies
```

## Roadmap

- [x] Milestone 0: project skeleton, CI, sanitizers, fuzz harness, wire format, simulated clock
- [x] Milestone 1: radar simulator, sensor table, single-sensor Kalman/EKF tracker, ground-truth regression tests (camera simulator moved to M2)
- [ ] Milestone 2: multi-sensor association and lifecycle, camera simulator, decision trace, record and replay, UDP ingest and daemon, WebSocket stream and map, ADS-B ingest. **Implemented; verification pending:**
  - [x] all C++ and Python tests pass, including under ASan/UBSan and TSan, built against small test-only stand-ins for Eigen and GoogleTest (the packages were not installable in the development sandbox)
  - [x] daemon end to end over UDP and WebSocket; live run and replay produce identical output
  - [ ] the same tests against the real `libeigen3-dev` / `libgtest-dev` (CI, or `cmake --preset dev` locally)
  - [ ] libFuzzer build of `fuzz_decode` and `fuzz_event_log` (the event-log target was only run under a GCC mutation driver)
  - [ ] the map in a real browser, and the OpenSky adapter against the live API (only tested offline on a sample written from the API documentation)
- [ ] Fuzzing, degraded-network tests, benchmarks, design document, demo video
- [ ] Stretch: CUDA-accelerated association

## Scope and ground rules

- Uses only public data and the project's own simulations. Nothing is based on controlled or non-public technical information.
- Focused on perception, tracking, and situational awareness. Nothing in the project involves weapons or targeting.
- Scope is cut before polish. A smaller system that works, is tested, and is clearly documented beats a large one that is half-finished.

## Data sources

Live aircraft data comes from the [OpenSky Network](https://opensky-network.org/). Use of that data is subject to the OpenSky Network's terms of use.

## License

Released under the [MIT License](LICENSE).
