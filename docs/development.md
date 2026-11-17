# Development guide

Everything below is for working on Sifted itself. To install and use it, see
the [README](../README.md).

## Building from source

Requires CMake 3.21+, Ninja, a C++20 compiler (GCC 13 or Clang 18 are tested;
the code needs C++20 floating-point `std::from_chars`, so GCC 11+ or Clang with
libc++ 20+), Eigen 3.4 and GoogleTest.

```bash
sudo apt install cmake ninja-build libeigen3-dev libgtest-dev   # Debian/Ubuntu

cmake --preset dev            # configure (Debug, warnings as errors)
cmake --build --preset dev
ctest --preset dev
```

Presets:

| Preset | What it is |
|---|---|
| `dev` | Debug, warnings as errors |
| `release` | Release, warnings as errors (benchmarks) |
| `asan` | ASan + UBSan |
| `tsan` | ThreadSanitizer |
| `fuzz` | Clang + libFuzzer + ASan/UBSan |
| `app` | what `install.sh` builds: Release, warnings not fatal, no tests or benchmarks (it turns tests on with `--with-tests`) |

No system packages? Add `-DSIFTED_FETCH_DEPS=ON` to download pinned copies of
Eigen and GoogleTest at configure time (needs git and internet). Downloaded
headers are included as system headers, so the project's strict warnings do not
apply to them. `install.sh` uses this automatically when Eigen can't be
installed from a package manager.

The socket code uses a few Linux extensions (`SOCK_CLOEXEC`, `accept4`,
`MSG_NOSIGNAL`) through `src/sifted/net/socket_compat.hpp`, which falls back to
plain POSIX calls on macOS and the BSDs. Define `SIFTED_PORTABLE_SOCKETS` to
compile the fallback on Linux, e.g.
`cmake -B build/portable -DCMAKE_CXX_FLAGS=-DSIFTED_PORTABLE_SOCKETS`.

**ThreadSanitizer on WSL2 and other recent kernels:** if every TSan binary
fails with "unexpected memory mapping", the kernel randomises mmap with more
bits than TSan supports. This is environmental, not a code bug. Run
`sudo sysctl vm.mmap_rnd_bits=28` once (until reboot), or run the tests
under `setarch "$(uname -m)" -R ctest --preset tsan`. CI does the former.

### Python tests

```bash
cd sim && PYTHONPATH=. python3 -m unittest discover -s tests
```

### Fuzzing

```bash
cmake --preset fuzz && cmake --build --preset fuzz     # needs libclang-rt-18-dev
mkdir -p build/fuzz-work/engine
build/fuzz/fuzz/fuzz_engine build/fuzz-work/engine fuzz/corpus_engine -max_total_time=60
```

Six targets (`fuzz_decode`, `fuzz_event_log`, `fuzz_log_header`,
`fuzz_sensor_table`, `fuzz_websocket`, `fuzz_engine`); see
[`fuzz/README.md`](../fuzz/README.md).

## Running the pieces by hand

The app launcher (`./sifted`, `app/launcher.py`) does all of this for you. To
run the daemon, simulators and map yourself:

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

Served this way (without the launcher), the operator view is a read-only track
display: the scenario, recording and control panels need the launcher's API.

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
The demo runbook is [`demo.md`](demo.md) (`scripts/demo.sh` runs it).

### Benchmarks

```bash
cmake --preset release && cmake --build --preset release
build/release/benchmarks/sifted_bench all --runs 5 > engine.csv     # ~15 min
python3 benchmarks/e2e_latency.py build/release/src/apps/sifted_fusiond > e2e.csv
python3 benchmarks/report.py <dir with the CSVs>                     # tables (+ plot)
```

Results and method: [`benchmarks.md`](benchmarks.md).

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

## The app layer

| File | Role |
|---|---|
| `install.sh` | Installer for Linux, macOS and WSL: detects the OS and package manager, installs missing tools, builds the `app` preset, runs an end-to-end smoke test (live run + replay compared), adds a shortcut. Idempotent. Log: `build/install.log`. |
| `install.cmd`, `scripts/windows/install.ps1` | Windows installer: checks/installs WSL and a distribution, normalises shell-script line endings, runs `install.sh` inside WSL, creates desktop and Start-menu shortcuts. Remembers the distribution in `build/wsl-distro.txt`. |
| `sifted`, `sifted.cmd` | Launchers with a visible terminal (Unix / Windows via WSL); both run `app/launcher.py`. |
| `scripts/windows/start.ps1` | What the Windows shortcut runs: starts the launcher hidden (`--background`), waits for `build/launcher.state` (a message box shows the end of `build/launcher.log` if it fails), then hosts the view in a native WinForms + WebView2 window with Sifted's icon and taskbar identity (AppUserModelID `Sifted.OperatorView`, also set on the shortcuts). Falls back to an Edge app window if WebView2 isn't available. `install.ps1` downloads the WebView2 SDK from NuGet into `build/webview2` and keeps it only if its DLLs carry a valid Microsoft signature. |
| `app/launcher.py` | Opens the view as a Chrome/Edge/Chromium/Brave app window (`--app=URL`) when one is installed, else a normal tab (`--browser` forces that). Only one copy runs: a second start opens another window on the first. With `--background` it logs to `build/launcher.log` and stops a few seconds after its last window closes (windows poll `/api/status` and send a `/api/bye` beacon on close). Standard-library Python. Serves `web/` and a small JSON control API on `127.0.0.1`, starts and stops the daemon, impairment proxy, sender and ADS-B adapter as child processes, records every run under the data folder and verifies replays. |
| `web/` | Operator view: `index.html`, `assets/app.js`, `assets/app.css`, bundled Leaflet in `vendor/leaflet/`. No build step. |

Control API (all JSON; `POST` requests need the header `X-Sifted: 1`, and the
`Host` must be the launcher's own address, which blocks cross-site requests
and DNS rebinding):

| Endpoint | |
|---|---|
| `GET /api/config` | version, track-stream URL, whether the daemon is built, network and fault presets |
| `GET /api/scenarios` | scenario names, descriptions, durations, sensors |
| `GET /api/status` | current activity, daemon counters, verify result, recent log lines |
| `GET /api/runs` | recorded runs (newest first; the last 25 are kept) |
| `POST /api/run` | `{scenario, speed, network: perfect\|lossy\|outage, fault: none\|restart\|dropout\|stuck}` |
| `POST /api/replay` | `{id, pace}`: replay a recording onto the map |
| `POST /api/verify` | `{id}`: replay as fast as possible and compare with the live output |
| `POST /api/adsb` | `{bbox: [south, north, west, east]}` |
| `POST /api/stop`, `/api/dismiss`, `/api/quit` | stop the activity, return to the start screen, exit |

Data folder: `~/.local/share/sifted` (Linux/WSL), `~/Library/Application
Support/Sifted` (macOS), or `$SIFTED_DATA_DIR`. Each run directory holds
`sensors.cfg`, `run.siftlog`, `live.txt`, `daemon.log` and `meta.json`.

## Layout

```
install.sh, install.cmd   installers (Linux/macOS/WSL, Windows)
sifted, sifted.cmd        launchers
app/          launcher (Python, standard library only) and app icons
src/sifted/   library: core (types, clock, sensor table, outcomes, multi-track engine),
              wire (codec), estimation (Kalman filter, EKF models, geodetic, gate screen),
              association (gating, assignment, lifecycle), store (track table,
              decision trace), replay (event log, replayer), net (ingest pipeline with
              dedup, clock gate and reorder stage, snapshots, impairment model, UDP,
              rate limiting, queue, WebSocket, socket portability)
src/apps/     sifted_fusiond, the fusion daemon (live and replay)
sim/          sensor simulators, scenarios, UDP sender with fault injection,
              impairment proxy, OpenSky adapter (Python)
web/          operator view (Leaflet bundled, no build step)
tests/        unit tests, fixtures, ground-truth regression, degraded-network,
              fault-injection and UDP flood tests
fuzz/         six libFuzzer targets and seed corpora
benchmarks/   engine benchmark harness, end-to-end latency script, report generator
scripts/      demo runner, Windows installer
docs/         design document, threat model, ADRs 0001-0011, benchmark report, demo runbook
cmake/        build options, sanitizers, dependencies
```

## Milestones

- **M0:** project skeleton, CI, sanitizers, fuzz harness, wire format, simulated clock.
- **M1:** radar simulator, sensor table, single-sensor Kalman/EKF tracker, ground-truth regression tests.
- **M2:** multi-sensor association and track lifecycle, camera simulator, decision trace, record and replay, UDP ingest and daemon, WebSocket stream and map, ADS-B ingest.
- **M3:** degraded-network simulator and recovery tests, bounded reorder stage, fault injection, clock gate and per-source rate limiting, fuzzing of every parser and the engine, benchmarks, design document, threat model, demo runbook ([`demo.md`](demo.md)).
- **Packaging:** one-step installers, app launcher with local control API, redesigned operator view with scenario runner, recordings and replay verification.
- **Possible next steps:** CUDA-accelerated association; a spatial index for more than ~3,000 tracks per engine thread; sensor bias estimation.
