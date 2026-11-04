# Sifted

**Signal, sifted.** Deterministic multi-sensor track fusion in C++.

Sifted takes noisy, partial, and sometimes conflicting measurements from many sensors and turns them into one coherent, trustworthy picture of what is out there. Simulated sensors and a live public ADS-B feed publish measurements to a central fusion service, which estimates each object's position and velocity, decides which measurements belong to which object, handles objects appearing and disappearing, and streams the resulting tracks to a live map.

It is a small-scale, from-scratch take on the core problem behind modern situational-awareness software. The goal is a working, well-tested, well-documented system that demonstrates systems-level engineering: determinism, traceability, robust handling of untrusted input, thread safety, networking, and measured performance.

> **Status: early development.** This README describes the design and the plan. Features are marked as built only when they are. See the [roadmap](#roadmap).

## Features

- **Sensor simulation.** Radar-like (range and bearing) and camera-like (bearing-only) sensors generate noisy measurements of moving objects, each with its own noise, update rate, and failure modes.
- **Real data feed.** Public ADS-B aircraft data (for example from the OpenSky Network) is ingested as an additional sensor, so the system runs on live, messy data and not only on simulation.
- **Track estimation.** Kalman-family filters (Kalman or extended Kalman) estimate each object's state and quantify the uncertainty of that estimate.
- **Data association.** Each measurement is matched to an existing track or starts a new one, using Mahalanobis gating. Tracks that stop receiving measurements are coasted, then dropped.
- **Operator view.** A web client subscribes to the track stream and shows a live map with track identity, confidence, and contributing sensors.
- **Deterministic record and replay.** Any run can be recorded and replayed so that identical input produces bit-for-bit identical output. Time is an input to the system, not read from the machine.
- **Traceability.** Every track can be traced back to the measurements and decisions that produced it, so you can ask why the system believes something.
- **Degraded-network behavior.** Packet loss, delay, reordering, and dropped links can be simulated, with measurements of how quickly and how well the picture recovers.
- **Fuzzing and fault injection.** Malformed packets, sensor dropouts, duplicate messages, and clock skew are injected automatically and run in CI.
- **Measured performance.** Latency, throughput, and tracks handled per core are benchmarked and reported.

## Architecture

```
 Simulated radar   --\
 Simulated camera  ---+--UDP-->  ingest + validate --> associate --> estimate --> track store --WebSocket--> operator map
 ADS-B (OpenSky)   --/                  |                                                |
                                        v                                                v
                                   event log  <------------- replay (simulated clock) ---+
```

Every message crosses a validated boundary: explicit versioning, size limits, and strict parsing on everything received. The core is multithreaded with clear ownership of shared state and is checked with thread sanitizers.

## Tech stack

| Area | Choice |
|---|---|
| Core | C++17/20, CMake |
| Math | Eigen (Kalman filters), nearest-neighbor gating with optional global nearest neighbor (Hungarian) |
| Networking | UDP for sensor data, TCP or WebSockets for operator clients, POSIX sockets or Boost.Asio |
| Message format | Protocol Buffers, FlatBuffers, or a small custom framing layer, versioned and size-limited |
| Record and replay | Append-only timestamped event log, simulated clock |
| Simulators and tooling | Python (sensor simulators, analysis, benchmark plots) |
| Operator display | HTML and JavaScript, Leaflet with OpenStreetMap tiles |
| Testing | GoogleTest or Catch2, ground-truth scenarios as a regression benchmark |
| Quality | libFuzzer or AFL++, ASan, UBSan, TSan, clang-tidy |
| CI | GitHub Actions |
| Optional | CUDA-accelerated association, benchmarked against the CPU path |

## Getting started

Build and run instructions will be added as the system comes together. The intended workflow:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## Planned layout

```
src/        fusion core, networking, replay engine
sim/        sensor simulators and scenario generators
web/        operator map client
tests/      unit, integration, and ground-truth scenario tests
fuzz/       fuzz targets for the parser and ingest paths
docs/       design document, threat model, benchmark report
```

## Roadmap

- [ ] Sensor simulator, message format, single-sensor tracker, first tests
- [ ] Multi-sensor association, networking, record and replay, basic live display
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
