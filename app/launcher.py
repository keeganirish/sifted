#!/usr/bin/env python3
"""Sifted app launcher: runs the whole system behind one command.

    ./sifted                      start, and open the operator view in a browser
    ./sifted --no-browser         start without opening a browser
    ./sifted --check              report what is installed and exit

It serves the operator view (web/) on http://127.0.0.1:PORT/ together with a
small local control API, and starts and stops the other pieces on demand:
the fusion daemon (sifted_fusiond), the scenario sender and impairment proxy
(sim/), and the OpenSky ADS-B adapter. Every run is recorded so it can be
replayed on the map or verified bit-for-bit against its replay.

Standard library only (Python 3.10+). Everything binds to 127.0.0.1; the
control API refuses requests from other origins.
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import filecmp
import http.server
import json
import os
import pathlib
import platform
import re
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import webbrowser

ROOT = pathlib.Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "sim"
WEB_DIR = ROOT / "web"
sys.path.insert(0, str(SIM_DIR))

try:
    from sifted_sim import generate as simgen  # noqa: E402  (stdlib-only package)
    from sifted_sim import send as simsend  # noqa: E402
except Exception as exc:  # pragma: no cover - only on a broken checkout
    raise SystemExit(f"sifted: cannot import the simulators in {SIM_DIR}: {exc}")

VERSION = "0.1.0"
HOST = "127.0.0.1"

# Friendly names for the built-in scenarios. Scenarios added to
# sim/sifted_sim/scenarios.py later still appear, with a generic description.
SCENARIO_INFO = {
    "demo": ("Guided Demo",
             "Five aircraft, one radar and one camera: a crossing pair, a target entering "
             "and one leaving coverage, and a turn. The best first run."),
    "crossing_pair": ("Crossing Pair",
                      "Two targets cross at a shallow angle. Association has to keep their "
                      "identities apart while they share a gate."),
    "close_pass": ("Close Pass",
                   "Two targets pass each other in lanes 120 m apart, plus a third that "
                   "manoeuvres elsewhere."),
    "formation_split": ("Formation Split",
                        "Three targets in close formation cross at a shallow angle, with "
                        "others entering, leaving and turning."),
    "enter_leave": ("Enter and Leave",
                    "Targets appear, vanish and leave coverage under heavier clutter and a "
                    "camera dropout: the full track lifecycle."),
    "straight_crossing": ("Bearing Seam",
                          "One target creeps across the radar's ±180° bearing seam, which "
                          "tests angle wrap-around."),
    "turn": ("Single Turn",
             "One target makes a 90° turn: manoeuvre detection in isolation."),
}
SCENARIO_ORDER = list(SCENARIO_INFO)

# Network conditions, as sifted_sim.impair arguments (ADR 0008). Times are
# wall-clock seconds after the proxy starts.
NETWORKS = {
    "perfect": ("Perfect Network", "Every frame arrives once, in order."),
    "lossy": ("Lossy Network", "3% loss with bursts, 50-200 ms delay with reordering, 2% duplicates."),
    "outage": ("Lossy + Outage", "As lossy, plus an 8-second link outage part-way through."),
}
LOSSY_ARGS = ["--loss", "0.03", "--burst", "0.01", "0.25", "0.6", "--delay-ms", "50",
              "--jitter-ms", "150", "--duplicate", "0.02"]

FAULTS = {
    "none": ("No Sensor Fault", ""),
    "restart": ("Radar Reboots", "The radar's sequence numbers restart a third of the way in."),
    "dropout": ("Radar Goes Silent", "The radar sends nothing for 10 s a third of the way in."),
    "stuck": ("Radar Freezes", "The radar repeats a stale scan for 8 s a third of the way in."),
}

ADSB_DEFAULT_BBOX = (51.0, 52.0, -1.2, 0.4)  # around London (README)
ADSB_SENSOR_ID = 100
ADSB_TRACKER_ARGS = ["--initial-velocity-sigma", "250", "--tentative-timeout", "90",
                     "--coast-after", "60", "--drop-after", "180", "--max-position-sigma", "10000"]

MAX_RUNS_KEPT = 25


# --------------------------------------------------------------------------- environment

def is_wsl() -> bool:
    try:
        return "microsoft" in pathlib.Path("/proc/version").read_text().lower()
    except OSError:
        return False


def default_data_dir() -> pathlib.Path:
    if os.environ.get("SIFTED_DATA_DIR"):
        return pathlib.Path(os.environ["SIFTED_DATA_DIR"]).expanduser()
    home = pathlib.Path.home()
    if sys.platform == "darwin":
        return home / "Library" / "Application Support" / "Sifted"
    base = os.environ.get("XDG_DATA_HOME") or str(home / ".local" / "share")
    return pathlib.Path(base) / "sifted"


def find_fusiond(explicit: str | None) -> pathlib.Path | None:
    candidates = []
    if explicit:
        candidates.append(pathlib.Path(explicit))
    if os.environ.get("SIFTED_FUSIOND"):
        candidates.append(pathlib.Path(os.environ["SIFTED_FUSIOND"]))
    for preset in ("app", "release", "dev", "asan"):
        candidates.append(ROOT / "build" / preset / "src" / "apps" / "sifted_fusiond")
    on_path = shutil.which("sifted_fusiond")
    if on_path:
        candidates.append(pathlib.Path(on_path))
    for c in candidates:
        if c.is_file() and os.access(c, os.X_OK):
            return c.resolve()
    return None


def port_free(port: int, kind: int) -> bool:
    s = socket.socket(socket.AF_INET, kind)
    try:
        if kind == socket.SOCK_STREAM:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def pick_port(preferred: int, kind: int, taken: set[int], tries: int = 50) -> int:
    for p in range(preferred, preferred + tries):
        if p not in taken and port_free(p, kind):
            taken.add(p)
            return p
    raise SystemExit(f"sifted: no free port near {preferred}; pass another with the options")


APP_WINDOW_ARGS = ["--window-size=1440,900"]


def _windows_path(win_path: str) -> pathlib.Path | None:
    """A Windows path as seen from WSL (C:\\... -> /mnt/c/...), or None."""
    try:
        out = subprocess.run(["wslpath", "-u", win_path], capture_output=True, text=True, timeout=5)
        return pathlib.Path(out.stdout.strip()) if out.returncode == 0 and out.stdout.strip() else None
    except (OSError, subprocess.TimeoutExpired):
        return None


def find_app_browser() -> list[str] | None:
    """A command that opens a URL as a standalone app window (Chrome, Edge,
    Chromium or Brave "app mode": no tabs or address bar), or None."""
    if is_wsl():
        for win in (r"C:\Program Files\Google\Chrome\Application\chrome.exe",
                    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
                    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
                    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"):
            p = _windows_path(win)
            if p and p.is_file():
                return [str(p)]
        return None
    if sys.platform == "darwin":
        for app in ("Google Chrome", "Microsoft Edge", "Chromium", "Brave Browser"):
            if pathlib.Path(f"/Applications/{app}.app").exists():
                return ["open", "-na", app, "--args"]
        return None
    for name in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser",
                 "microsoft-edge", "microsoft-edge-stable", "brave-browser"):
        exe = shutil.which(name)
        if exe:
            return [exe]
    return None


def open_app_window(url: str) -> bool:
    """Opens the operator view in its own app window. False if no suitable browser."""
    if os.environ.get("SIFTED_NO_BROWSER"):
        return True
    cmd = find_app_browser()
    if not cmd:
        return False
    try:
        cwd = str(pathlib.Path(cmd[0]).parent) if is_wsl() else None  # not a UNC path for Windows
        subprocess.Popen([*cmd, f"--app={url}", *APP_WINDOW_ARGS], cwd=cwd,
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL, start_new_session=True)
        return True
    except OSError:
        return False


def open_browser(url: str) -> None:
    if os.environ.get("SIFTED_NO_BROWSER"):
        return
    if is_wsl():
        # The Windows browser reaches WSL's localhost through WSL's port forwarding.
        for cmd in (["wslview", url], ["cmd.exe", "/c", "start", "", url], ["explorer.exe", url]):
            if shutil.which(cmd[0]):
                try:
                    subprocess.Popen(cmd, cwd="/", stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
                    return
                except OSError:
                    continue
    try:
        webbrowser.open(url)
    except Exception:
        pass


# --------------------------------------------------------------------------- scenarios

def parse_sensors(text: str) -> list[dict]:
    out = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] == "sensor":
            out.append({"id": int(parts[1]), "kind": parts[2]})
    return out


class Scenarios:
    """Generates scenario files on demand into the data directory, and describes them."""

    def __init__(self, data_dir: pathlib.Path):
        self.dir = data_dir / "scenarios"
        self._meta: dict[str, dict] = {}
        self._lock = threading.Lock()

    def names(self) -> list[str]:
        known = [n for n in SCENARIO_ORDER if n in simgen.ALL]
        return known + sorted(n for n in simgen.ALL if n not in SCENARIO_INFO)

    def materialise(self, name: str) -> pathlib.Path:
        if name not in simgen.ALL:
            raise ValueError(f"unknown scenario {name!r}")
        out = self.dir / name
        # Regenerated every time: cheap, and always matches the simulator code.
        simgen.write(simgen.ALL[name](), out)
        return out

    def meta(self, name: str) -> dict:
        with self._lock:
            if name not in self._meta:
                d = self.materialise(name)
                sensors = parse_sensors((d / "sensors.cfg").read_text())
                frames = simsend.load_frames(str(d))
                times = [simsend.frame_time_ns(f) for f in frames] or [0]
                targets: set[str] = set()
                truth = d / "truth.csv"
                if truth.is_file():
                    with open(truth) as f:
                        header = next(f, "").strip().split(",")
                        col = header.index("target_id") if "target_id" in header else None
                        for line in f:
                            cols = line.strip().split(",")
                            targets.add(cols[col] if col is not None and col < len(cols) else "1")
                title, desc = SCENARIO_INFO.get(
                    name, (name.replace("_", " ").title(), "Simulated scenario."))
                self._meta[name] = {
                    "name": name, "title": title, "description": desc,
                    "duration_s": round((max(times) - min(times)) / 1e9, 1),
                    "targets": len(targets), "frames": len(frames), "sensors": sensors,
                }
            return self._meta[name]


# --------------------------------------------------------------------------- processes

def parse_stats(line: str) -> dict[str, int]:
    """Counters from a sifted_fusiond stats line ("udp received 10, rejected 0, ...")."""
    body = line.split(":", 1)[1] if ":" in line else line
    body = body.replace("(", ", ").replace(")", "").replace(";", ",")
    out = {}
    for part in body.split(","):
        m = re.match(r"\s*(?:udp )?([a-z][a-z -]*?)\s+(\d+)", part)
        if m:
            out[m.group(1).strip().replace(" ", "_").replace("-", "_")] = int(m.group(2))
    return out


class Proc:
    """A child process whose stderr/stdout lines go to the session log."""

    def __init__(self, name: str, args: list[str], session: "Session", cwd: pathlib.Path,
                 logfile: pathlib.Path | None = None, env: dict | None = None):
        self.name = name
        self.session = session
        self.logfile = open(logfile, "a", encoding="utf-8") if logfile else None
        self.ready = threading.Event()
        self.popen = subprocess.Popen(
            args, cwd=cwd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1, env=env,
            start_new_session=True)  # Ctrl-C in the terminal reaches only the launcher
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self) -> None:
        assert self.popen.stdout is not None
        for raw in self.popen.stdout:
            line = raw.rstrip("\n")
            if self.logfile:
                self.logfile.write(line + "\n")
                self.logfile.flush()
            self.session.on_line(self, line)
        if self.logfile:
            self.logfile.close()

    def alive(self) -> bool:
        return self.popen.poll() is None

    def stop(self, timeout: float = 5.0) -> None:
        """Polite signal first, then escalate. The daemon gets SIGINT, on which it
        drains its queue and flushes the event log and output; the Python tools
        get SIGTERM, which ends them without a traceback."""
        if not self.alive():
            return
        first = signal.SIGINT if self.name == "daemon" else signal.SIGTERM
        for sig, wait in ((first, timeout), (signal.SIGTERM, 2.0), (signal.SIGKILL, 2.0)):
            try:
                self.popen.send_signal(sig)
            except ProcessLookupError:
                return
            try:
                self.popen.wait(wait)
                break
            except subprocess.TimeoutExpired:
                continue
        self.reader.join(timeout=2.0)


# --------------------------------------------------------------------------- session

class SessionError(Exception):
    pass


class Session:
    """What is running now. One activity at a time: a run, a replay or live ADS-B."""

    def __init__(self, cfg: argparse.Namespace, fusiond: pathlib.Path | None):
        self.cfg = cfg
        self.fusiond = fusiond
        self.data_dir: pathlib.Path = cfg.data_dir
        self.runs_dir = self.data_dir / "runs"
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        self.scenarios = Scenarios(self.data_dir)
        self.lock = threading.RLock()
        self.procs: dict[str, Proc] = {}
        self.log: collections.deque = collections.deque(maxlen=300)
        # idle | starting | running | finished (all sent) | complete (run stopped,
        # summary kept) | replaying | live | stopping
        self.state = "idle"
        self.activity: dict = {}
        self.stats: dict[str, int] = {}
        self.error: str | None = None
        self.verify: dict = {}
        self.seq = 0            # bumps on every state change, so the UI can tell

    # ---- helpers

    def note(self, text: str, level: str = "info") -> None:
        stamp = dt.datetime.now().strftime("%H:%M:%S")
        self.log.append({"t": stamp, "level": level, "text": text})

    def _bump(self, state: str | None = None) -> None:
        if state:
            self.state = state
        self.seq += 1

    def on_line(self, proc: Proc, line: str) -> None:
        # Set outside the lock: the thread starting this process holds it while it waits.
        if proc.name == "daemon" and ("track snapshots on ws://" in line
                                      or "listening on udp" in line):
            proc.ready.set()
        with self.lock:
            if proc.name == "daemon" and ("udp received" in line or line.startswith(
                    "sifted_fusiond: frames ")):
                self.stats.update(parse_stats(line))
                return  # counters go to the stats panel, not the activity log
            if proc.name == "daemon" and "replay finished" in line:
                self._replay_done()
            if proc.name == "adsb":
                if "next attempt" in line:
                    self.error = ("Couldn't reach OpenSky (" + line.removeprefix("adsb: ") +
                                  "). Check your internet connection; Sifted keeps retrying.")
                    self._bump()
                elif " sent, " in line and self.error:
                    self.error = None
                    self._bump()
            level = "error" if re.search(r"error|cannot|invalid|failed|Traceback", line, re.I) \
                else "info"
            text = line.removeprefix("sifted_fusiond: ").removeprefix(f"{proc.name}: ")
            self.note(f"{proc.name}: {text}", level)

    def _require_daemon(self) -> pathlib.Path:
        if self.fusiond is None:
            raise SessionError("The fusion service is not built yet. Run the installer "
                               "(./install.sh, or install.cmd on Windows) and start Sifted again.")
        return self.fusiond

    def _python(self) -> list[str]:
        return [sys.executable, "-u"]

    def _env(self) -> dict:
        env = dict(os.environ)
        env["PYTHONPATH"] = str(SIM_DIR) + os.pathsep + env.get("PYTHONPATH", "")
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def _start_daemon(self, args: list[str], logfile: pathlib.Path | None) -> Proc:
        exe = self._require_daemon()
        full = [str(exe), *args, "--ws-port", str(self.cfg.ws_port), "--ws-bind", HOST]
        p = Proc("daemon", full, self, cwd=ROOT, logfile=logfile)
        self.procs["daemon"] = p
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not p.ready.is_set() and p.alive():
            time.sleep(0.05)
        if not p.alive() and not p.ready.is_set():
            p.reader.join(timeout=1.0)
            last = [e["text"] for e in list(self.log)[-4:] if e["text"].startswith("daemon")]
            hint = " ".join(last) or "it exited immediately"
            if "Address already in use" in hint:
                hint += (" - another program (or another copy of Sifted) is using the port; "
                         "close it or start Sifted with --ws-port / --udp-port")
            raise SessionError(f"The fusion service failed to start: {hint}")
        return p

    def _stop_all_locked(self) -> None:
        # Senders first (so nothing arrives at a stopping daemon), daemon last.
        for name in ("adsb", "sender", "proxy", "daemon"):
            p = self.procs.pop(name, None)
            if p:
                p.stop()
        if self.state in ("running", "finished", "live", "stopping"):
            self._finalise_run()

    def _finalise_run(self) -> None:
        run = self.activity.get("run_dir")
        if not run:
            return
        meta_path = pathlib.Path(run) / "meta.json"
        try:
            meta = json.loads(meta_path.read_text())
            meta["finished"] = dt.datetime.now().isoformat(timespec="seconds")
            if "t0" in self.activity and self.activity.get("speed"):
                # Scenario seconds actually sent, so a replay can show its progress.
                meta["recorded_s"] = round(min(
                    (time.monotonic() - self.activity["t0"]) * self.activity["speed"],
                    self.activity.get("duration_s") or float("inf")), 1)
            meta["stats"] = self.stats
            meta["complete"] = True
            meta_path.write_text(json.dumps(meta, indent=2))
        except (OSError, ValueError):
            pass

    def _prune_runs(self) -> None:
        runs = sorted((d for d in self.runs_dir.iterdir() if (d / "meta.json").is_file()),
                      key=lambda d: d.name)
        for d in runs[:-MAX_RUNS_KEPT]:
            shutil.rmtree(d, ignore_errors=True)

    # ---- actions

    def start_run(self, scenario: str, speed: float, network: str, fault: str) -> None:
        if scenario not in simgen.ALL:
            raise SessionError(f"Unknown scenario {scenario!r}.")
        if network not in NETWORKS:
            raise SessionError(f"Unknown network condition {network!r}.")
        if fault not in FAULTS:
            raise SessionError(f"Unknown fault {fault!r}.")
        if not (0.25 <= speed <= 50):
            raise SessionError("Speed must be between 0.25x and 50x.")
        meta = self.scenarios.meta(scenario)
        duration = meta["duration_s"]
        radars = [s["id"] for s in meta["sensors"] if s["kind"] == "range_bearing"]
        fault_spec = None
        if fault != "none":
            if not radars:
                raise SessionError("This scenario has no radar to apply the fault to.")
            a = round(duration / 3)
            fault_spec = {"restart": f"restart:{radars[0]}@{a}",
                          "dropout": f"dropout:{radars[0]}@{a}-{a + 10}",
                          "stuck": f"stuck:{radars[0]}@{a}-{a + 8}"}[fault]
        with self.lock:
            self._require_daemon()
            self._stop_all_locked()
            self.error, self.stats, self.verify = None, {}, {}
            self._bump("starting")
            sdir = self.scenarios.materialise(scenario)
            stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
            run_dir = self.runs_dir / f"{stamp}-{scenario}"
            n = 1
            while run_dir.exists():
                n += 1
                run_dir = self.runs_dir / f"{stamp}-{scenario}-{n}"
            run_dir.mkdir(parents=True)
            shutil.copy(sdir / "sensors.cfg", run_dir / "sensors.cfg")
            info = {
                "id": run_dir.name, "scenario": scenario, "title": meta["title"],
                "started": dt.datetime.now().isoformat(timespec="seconds"),
                "speed": speed, "network": network, "fault": fault, "fault_spec": fault_spec,
                "duration_s": duration, "complete": False,
            }
            (run_dir / "meta.json").write_text(json.dumps(info, indent=2))
            self.note(f"Starting {meta['title']} ({speed:g}x, {NETWORKS[network][0].lower()}"
                      f"{', ' + FAULTS[fault][0].lower() if fault != 'none' else ''})")
            try:
                self._start_daemon(
                    ["--sensors", str(run_dir / "sensors.cfg"),
                     "--listen", f"{HOST}:{self.cfg.udp_port}",
                     "--record", str(run_dir / "run.siftlog"),
                     "--output", str(run_dir / "live.txt"),
                     "--stats-interval", "1"],
                    run_dir / "daemon.log")
                target = f"{HOST}:{self.cfg.udp_port}"
                if network != "perfect":
                    proxy_args = ["--listen", f"{HOST}:{self.cfg.proxy_port}", "--target", target,
                                  "--seed", "7", *LOSSY_ARGS]
                    if network == "outage":
                        a = duration * 0.47
                        proxy_args += ["--outage", f"{a / speed:.2f}:{(a + 8) / speed:.2f}"]
                        info["outage_at_s"] = round(a, 1)
                    self.procs["proxy"] = Proc(
                        "proxy", [*self._python(), "-m", "sifted_sim.impair", *proxy_args],
                        self, cwd=SIM_DIR, env=self._env())
                    target = f"{HOST}:{self.cfg.proxy_port}"
                    time.sleep(0.3)
                send_args = [*self._python(), "-m", "sifted_sim.send", str(sdir),
                             "--target", target, "--speed", f"{speed:g}"]
                if fault_spec:
                    send_args += ["--fault", fault_spec]
                self.procs["sender"] = Proc("sender", send_args, self, cwd=SIM_DIR,
                                            env=self._env())
            except SessionError as e:
                self._stop_all_locked()
                self.error = str(e)
                self._bump("idle")
                raise
            info["run_dir"] = str(run_dir)
            info["t0"] = time.monotonic()
            self.activity = info
            self._prune_runs()
            self._bump("running")
            threading.Thread(target=self._watch_sender, args=(self.procs["sender"],),
                             daemon=True).start()

    def _watch_sender(self, sender: Proc) -> None:
        rc = sender.popen.wait()
        sender.reader.join(timeout=2.0)
        with self.lock:
            if self.procs.get("sender") is not sender:
                return  # superseded by another action
            self.procs.pop("sender", None)
            if rc != 0:
                self.error = "The scenario sender stopped with an error; see the activity log."
            self.note("All frames sent. Tracks will coast and drop out as their targets go "
                      "quiet. Stop the run to finish the recording.")
            self._bump("finished")

    def start_replay(self, run_id: str, pace: float) -> None:
        with self.lock:
            self._require_daemon()
            run_dir = self._run_dir(run_id)
            if not (run_dir / "run.siftlog").is_file():
                raise SessionError("That run has no recording.")
            if self.activity.get("id") == run_id and self.state in ("running", "finished"):
                self.note("Stopping the live run to finish its recording")
            self._stop_all_locked()
            self.error, self.stats, self.verify = None, {}, {}
            meta = json.loads((run_dir / "meta.json").read_text())
            replay_out = run_dir / "replay.txt"
            replay_out.unlink(missing_ok=True)
            self._bump("starting")
            self.note(f"Replaying {meta.get('title', run_id)} at {pace:g}x")
            try:
                self._start_daemon(["--replay", str(run_dir / "run.siftlog"), "--pace",
                                    f"{pace:g}", "--hold", "--output", str(replay_out)],
                                   run_dir / "replay.log")
            except SessionError as e:
                self.error = str(e)
                self._bump("idle")
                raise
            self.activity = {**meta, "run_dir": str(run_dir), "pace": pace,
                             "t0": time.monotonic(), "replay_done": False}
            self._bump("replaying")

    def _replay_done(self) -> None:
        run_dir = pathlib.Path(self.activity.get("run_dir", ""))
        self.activity["replay_done"] = True
        live, rep = run_dir / "live.txt", run_dir / "replay.txt"
        if live.is_file() and rep.is_file():
            same = filecmp.cmp(live, rep, shallow=False)
            self.activity["identical"] = same
            self.note("Replay finished: output is bit-for-bit identical to the live run"
                      if same else "Replay finished, but its output differs from the live run",
                      "info" if same else "error")
        self._bump()

    def verify_run(self, run_id: str) -> None:
        """Replays a run as fast as possible (no map) and compares outputs, in the background."""
        with self.lock:
            exe = self._require_daemon()
            run_dir = self._run_dir(run_id)
            if self.activity.get("id") == run_id and self.state in ("running", "finished"):
                self._complete_locked()
            self.verify = {"id": run_id, "state": "running"}
            self._bump()

        def work() -> None:
            out = run_dir / "verify.txt"
            t0 = time.monotonic()
            r = subprocess.run([str(exe), "--replay", str(run_dir / "run.siftlog"), "--output",
                                str(out), "--ws-port", "0"], capture_output=True, text=True)
            took = time.monotonic() - t0
            live = run_dir / "live.txt"
            with self.lock:
                if r.returncode != 0:
                    self.verify = {"id": run_id, "state": "error",
                                   "message": (r.stderr.strip().splitlines() or ["failed"])[-1]}
                elif not live.is_file():
                    self.verify = {"id": run_id, "state": "error",
                                   "message": "the live run wrote no output"}
                else:
                    same = filecmp.cmp(live, out, shallow=False)
                    lines = sum(1 for _ in open(out, encoding="utf-8", errors="replace"))
                    self.verify = {"id": run_id, "state": "identical" if same else "different",
                                   "lines": lines, "seconds": round(took, 2)}
                    self.note(f"Verified {run_id}: replay "
                              f"{'identical' if same else 'DIFFERS'} ({lines} output lines, "
                              f"{took:.2f} s)", "info" if same else "error")
                out.unlink(missing_ok=True)
                self._bump()

        threading.Thread(target=work, daemon=True).start()

    def start_adsb(self, bbox: tuple[float, float, float, float]) -> None:
        lamin, lamax, lomin, lomax = bbox
        if not (-90 <= lamin < lamax <= 90 and -180 <= lomin < lomax <= 180):
            raise SessionError("The area must be south < north latitude and west < east "
                               "longitude.")
        if (lamax - lamin) * (lomax - lomin) > 25:
            raise SessionError("Choose a smaller area (at most about 5° x 5°) to stay within "
                               "OpenSky's free quota.")
        with self.lock:
            self._require_daemon()
            self._stop_all_locked()
            self.error, self.stats, self.verify = None, {}, {}
            self._bump("starting")
            stamp = dt.datetime.now().strftime("%Y%m%d-%H%M%S")
            run_dir = self.runs_dir / f"{stamp}-adsb"
            run_dir.mkdir(parents=True, exist_ok=True)
            sensors = run_dir / "sensors.cfg"
            subprocess.run([*self._python(), "-m", "sifted_sim.adsb", "--write-sensors",
                            str(sensors), "--sensor-id", str(ADSB_SENSOR_ID)],
                           cwd=SIM_DIR, env=self._env(), check=True, capture_output=True)
            origin = f"{(lamin + lamax) / 2:.4f},{(lomin + lomax) / 2:.4f}"
            info = {"id": run_dir.name, "scenario": "adsb", "title": "Live Aircraft (ADS-B)",
                    "started": dt.datetime.now().isoformat(timespec="seconds"),
                    "bbox": list(bbox), "complete": False}
            (run_dir / "meta.json").write_text(json.dumps(info, indent=2))
            authed = bool(os.environ.get("OPENSKY_CLIENT_ID"))
            self.note("Starting live ADS-B from the OpenSky Network "
                      f"({'authenticated' if authed else 'anonymous: first data can take a few minutes'})")
            try:
                self._start_daemon(["--sensors", str(sensors), "--listen",
                                    f"{HOST}:{self.cfg.udp_port}", "--origin", origin,
                                    *ADSB_TRACKER_ARGS, "--record", str(run_dir / "run.siftlog"),
                                    "--output", str(run_dir / "live.txt"),
                                    "--stats-interval", "2"], run_dir / "daemon.log")
                self.procs["adsb"] = Proc(
                    "adsb", [*self._python(), "-m", "sifted_sim.adsb", "--bbox",
                             ",".join(f"{v:g}" for v in bbox), "--sensor-id",
                             str(ADSB_SENSOR_ID), "--target", f"{HOST}:{self.cfg.udp_port}"],
                    self, cwd=SIM_DIR, env=self._env())
            except SessionError as e:
                self._stop_all_locked()
                self.error = str(e)
                self._bump("idle")
                raise
            info["run_dir"] = str(run_dir)
            info["t0"] = time.monotonic()
            self.activity = info
            self._bump("live")
            threading.Thread(target=self._watch_adsb, args=(self.procs["adsb"],),
                             daemon=True).start()

    def _watch_adsb(self, proc: Proc) -> None:
        rc = proc.popen.wait()
        with self.lock:
            if self.procs.get("adsb") is not proc:
                return
            self.procs.pop("adsb", None)
            self.error = ("The ADS-B adapter stopped" + (f" (exit code {rc})" if rc else "") +
                          ". Check your internet connection and the activity log.")
            self._bump()

    def _complete_locked(self) -> None:
        """Stops a scenario run and keeps its summary on screen ("complete")."""
        if "t0" in self.activity:
            self.activity["elapsed_final"] = time.monotonic() - self.activity.pop("t0")
        self._stop_all_locked()
        self.activity["complete"] = True
        self._bump("complete")

    def stop(self) -> None:
        with self.lock:
            if self.state in ("idle", "complete"):
                return
            if self.state in ("running", "finished"):
                self._complete_locked()
                self.note("Run stopped; its recording is complete")
                return
            self._bump("stopping")
            self._stop_all_locked()
            self.note("Stopped")
            self.activity = {}
            self._bump("idle")

    def dismiss(self) -> None:
        """Leaves the summary of a finished run or replay and returns to the start screen."""
        with self.lock:
            self._stop_all_locked()
            self.activity, self.stats = {}, {}
            self._bump("idle")

    def shutdown(self) -> None:
        with self.lock:
            self._stop_all_locked()

    # ---- queries

    def _run_dir(self, run_id: str) -> pathlib.Path:
        if not re.fullmatch(r"[A-Za-z0-9_.-]+", run_id or ""):
            raise SessionError("Bad run id.")
        d = self.runs_dir / run_id
        if not (d / "meta.json").is_file():
            raise SessionError("That run no longer exists.")
        return d

    def runs(self) -> list[dict]:
        out = []
        for d in sorted(self.runs_dir.iterdir(), reverse=True):
            try:
                meta = json.loads((d / "meta.json").read_text())
            except (OSError, ValueError):
                continue
            log = d / "run.siftlog"
            meta["size_kb"] = round(log.stat().st_size / 1024) if log.is_file() else 0
            meta["recording"] = log.is_file()
            meta.pop("run_dir", None)
            out.append(meta)
        return out

    def status(self) -> dict:
        with self.lock:
            act = {k: v for k, v in self.activity.items()
                   if k not in ("t0", "run_dir", "elapsed_final")}
            if "t0" in self.activity or "elapsed_final" in self.activity:
                elapsed = self.activity.get("elapsed_final") or \
                    time.monotonic() - self.activity["t0"]
                act["elapsed_s"] = round(elapsed, 1)
                replay = self.state == "replaying"
                rate = (self.activity.get("pace") if replay else self.activity.get("speed")) or 0
                span = (self.activity.get("recorded_s") if replay else None) or \
                    self.activity.get("duration_s")
                if span and rate:
                    act["progress"] = 1.0 if self.activity.get("replay_done") else \
                        min(1.0, elapsed * rate / span)
                act["rate"] = rate
            return {"state": self.state, "seq": self.seq, "activity": act, "stats": self.stats,
                    "error": self.error, "verify": self.verify, "log": list(self.log)[-80:]}


# --------------------------------------------------------------------------- HTTP

class Handler(http.server.SimpleHTTPRequestHandler):
    server_version = "Sifted/" + VERSION
    session: Session
    app: "App"

    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(WEB_DIR), **kw)

    def log_message(self, fmt, *args):  # quiet: the terminal is for the user
        if self.app.cfg.verbose:
            super().log_message(fmt, *args)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()

    def _host_ok(self) -> bool:
        host = (self.headers.get("Host") or "").lower()
        return host in self.app.allowed_hosts

    def _json(self, code: int, obj) -> None:
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if not self._host_ok():  # DNS-rebinding guard
            self.send_error(403, "Forbidden host")
            return
        path = self.path.split("?", 1)[0]
        if path.startswith("/api/"):
            try:
                self._json(200, self.app.get(path))
            except KeyError:
                self._json(404, {"error": "not found"})
            except SessionError as e:
                self._json(400, {"error": str(e)})
            return
        super().do_GET()

    def do_POST(self):
        if self.path == "/api/bye":
            origin = self.headers.get("Origin")
            ok = self._host_ok() and (not origin or origin.lower().split("//", 1)[-1]
                                      in self.app.allowed_hosts)
            if ok:
                self.app.bye_at = time.monotonic()
            self.send_response(204 if ok else 403)
            self.end_headers()
            return
        # Cross-site requests cannot set this header without a CORS preflight,
        # which this server never grants.
        if not self._host_ok() or self.headers.get("X-Sifted") != "1":
            self._json(403, {"error": "forbidden"})
            return
        try:
            n = min(int(self.headers.get("Content-Length") or 0), 64 * 1024)
            body = json.loads(self.rfile.read(n) or b"{}")
            if not isinstance(body, dict):
                raise ValueError
        except ValueError:
            self._json(400, {"error": "expected a JSON object"})
            return
        try:
            self._json(200, self.app.post(self.path, body))
        except KeyError:
            self._json(404, {"error": "not found"})
        except SessionError as e:
            self._json(409, {"error": str(e)})
        except (TypeError, ValueError) as e:
            self._json(400, {"error": f"bad request: {e}"})
        except Exception as e:  # pragma: no cover - surfaced to the UI instead of a hang
            self._json(500, {"error": f"internal error: {e}"})


class App:
    def __init__(self, cfg: argparse.Namespace, fusiond: pathlib.Path | None):
        self.cfg = cfg
        self.session = Session(cfg, fusiond)
        self.allowed_hosts = {f"127.0.0.1:{cfg.port}", f"localhost:{cfg.port}"}
        self.quit = threading.Event()
        self.started = time.monotonic()
        self.last_seen: float | None = None   # last status poll from any open window
        self.bye_at: float | None = None      # last "window closed" beacon

    def watch_windows(self) -> None:
        """--background: quit once no window is open (closing the window quits Sifted).
        Windows poll /api/status every second (at least once a minute when hidden or
        minimised) and send a beacon when closed."""
        while not self.quit.wait(1.0):
            now = time.monotonic()
            if self.last_seen is None:
                if now - self.started > 120:
                    print("sifted: no window connected within 2 minutes; stopping", flush=True)
                    self.quit.set()
            elif self.bye_at and self.bye_at > self.last_seen and now - self.bye_at > 4:
                print("sifted: the window was closed; stopping", flush=True)
                self.quit.set()
            elif now - self.last_seen > 180:
                print("sifted: no window has been seen for 3 minutes; stopping", flush=True)
                self.quit.set()

    def config(self) -> dict:
        return {
            "version": VERSION,
            "ws_url": f"ws://{{host}}:{self.cfg.ws_port}/",
            "built": self.session.fusiond is not None,
            "platform": f"{platform.system()}{' (WSL)' if is_wsl() else ''}",
            "data_dir": str(self.cfg.data_dir),
            "networks": [{"id": k, "title": v[0], "description": v[1]} for k, v in NETWORKS.items()],
            "faults": [{"id": k, "title": v[0], "description": v[1]} for k, v in FAULTS.items()],
            "adsb_bbox": ADSB_DEFAULT_BBOX,
            "adsb_authenticated": bool(os.environ.get("OPENSKY_CLIENT_ID")),
        }

    def get(self, path: str):
        if path == "/api/config":
            return self.config()
        if path == "/api/status":
            self.last_seen = time.monotonic()
            return self.session.status()
        if path == "/api/scenarios":
            return [self.session.scenarios.meta(n) for n in self.session.scenarios.names()]
        if path == "/api/runs":
            return self.session.runs()
        raise KeyError(path)

    def post(self, path: str, body: dict):
        s = self.session
        if path == "/api/run":
            s.start_run(str(body.get("scenario", "demo")), float(body.get("speed", 1)),
                        str(body.get("network", "perfect")), str(body.get("fault", "none")))
        elif path == "/api/replay":
            s.start_replay(str(body["id"]), float(body.get("pace", 1)))
        elif path == "/api/verify":
            s.verify_run(str(body["id"]))
        elif path == "/api/adsb":
            bbox = body.get("bbox", ADSB_DEFAULT_BBOX)
            if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
                raise ValueError("bbox must be [south, north, west, east]")
            s.start_adsb(tuple(float(v) for v in bbox))
        elif path == "/api/stop":
            s.stop()
        elif path == "/api/dismiss":
            s.dismiss()
        elif path == "/api/quit":
            threading.Thread(target=lambda: (time.sleep(0.2), self.quit.set()), daemon=True).start()
            return {"ok": True}
        else:
            raise KeyError(path)
        return s.status()


# --------------------------------------------------------------------------- main

def check(cfg: argparse.Namespace) -> int:
    ok = True

    def line(good: bool, what: str, detail: str) -> None:
        nonlocal ok
        ok = ok and good
        print(f"  [{'ok' if good else '!!'}] {what:<18} {detail}")

    print(f"Sifted {VERSION} - environment check")
    py = sys.version_info
    line(py >= (3, 10), "Python", f"{platform.python_version()} ({sys.executable})")
    exe = find_fusiond(cfg.fusiond)
    line(exe is not None, "Fusion service", str(exe) if exe else
         "not built - run ./install.sh (install.cmd on Windows)")
    if exe:
        r = subprocess.run([str(exe), "--version"], capture_output=True, text=True)
        line(r.returncode == 0, "  runs", (r.stdout or r.stderr).strip())
    line((WEB_DIR / "index.html").is_file(), "Operator view", str(WEB_DIR))
    line(bool(simgen.ALL), "Simulators", f"{len(simgen.ALL)} scenarios")
    for name, port, kind in (("Web port", cfg.port, socket.SOCK_STREAM),
                             ("WebSocket port", cfg.ws_port, socket.SOCK_STREAM),
                             ("UDP port", cfg.udp_port, socket.SOCK_DGRAM)):
        free = port_free(port, kind)
        print(f"  [{'ok' if free else '--'}] {name:<18} {port} "
              f"{'free' if free else 'in use (Sifted will pick the next free one)'}")
    print(f"  [ok] {'Data folder':<18} {cfg.data_dir}")
    print(f"  [ok] {'Platform':<18} {platform.system()} {platform.machine()}"
          f"{' (WSL)' if is_wsl() else ''}")
    print("All good." if ok else "Some checks failed; see above.")
    return 0 if ok else 1


def find_running(port: int) -> str | None:
    """URL of a Sifted launcher already serving on `port`, if there is one."""
    import urllib.request
    url = f"http://{HOST}:{port}/"
    try:
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))  # never via a proxy
        with opener.open(url + "api/config", timeout=1.0) as r:
            cfg = json.loads(r.read(64 * 1024))
        return url if isinstance(cfg, dict) and "ws_url" in cfg and "version" in cfg else None
    except Exception:
        return None


def write_state(path: pathlib.Path, text: str) -> None:
    """Tells the Windows starter (scripts/windows/start.ps1) that Sifted is up."""
    try:
        path.parent.mkdir(exist_ok=True)
        path.write_text(text + "\n", encoding="ascii")
    except OSError:
        pass


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted", description="Start Sifted and open the operator view.")
    ap.add_argument("--port", type=int, default=int(os.environ.get("SIFTED_PORT", 8000)),
                    help="web port [8000, or the next free one]")
    ap.add_argument("--ws-port", type=int, default=8765, help="track stream port [8765]")
    ap.add_argument("--udp-port", type=int, default=5600, help="sensor UDP port [5600]")
    ap.add_argument("--proxy-port", type=int, default=5601, help="impairment proxy port [5601]")
    ap.add_argument("--fusiond", help="path to sifted_fusiond (default: the build folder)")
    ap.add_argument("--data-dir", type=pathlib.Path, default=default_data_dir(),
                    help=f"where scenarios and recordings are kept [{default_data_dir()}]")
    ap.add_argument("--no-browser", action="store_true", help="do not open a browser")
    ap.add_argument("--browser", action="store_true",
                    help="open a normal browser tab instead of an app window")
    ap.add_argument("--background", action="store_true",
                    help="run without a terminal: log to build/launcher.log and stop when "
                         "the app window is closed (used by the desktop shortcuts)")
    ap.add_argument("--run", metavar="SCENARIO", help="start this scenario right away (e.g. demo)")
    ap.add_argument("--check", action="store_true", help="check the installation and exit")
    ap.add_argument("--verbose", action="store_true", help="log every HTTP request")
    ap.add_argument("--version", action="version", version=f"sifted {VERSION}")
    cfg = ap.parse_args(argv)
    cfg.data_dir = cfg.data_dir.expanduser().resolve()

    if sys.version_info < (3, 10):
        print("sifted: Python 3.10 or newer is required", file=sys.stderr)
        return 1
    if cfg.check:
        return check(cfg)

    state_file = ROOT / "build" / "launcher.state"
    if cfg.background:
        # No terminal: everything goes to build/launcher.log (fresh each start).
        try:
            (ROOT / "build").mkdir(exist_ok=True)
            log = open(ROOT / "build" / "launcher.log", "w", buffering=1, encoding="utf-8")
            sys.stdout = sys.stderr = log
        except OSError:
            pass

    # Already running? Bring up another window on it instead of a second copy.
    existing = find_running(cfg.port)
    if existing:
        print(f"sifted: already running at {existing}; opening a window", flush=True)
        if not cfg.no_browser and (cfg.browser or not open_app_window(existing)):
            open_browser(existing)
        write_state(state_file, f"running {existing}")
        return 0

    taken: set[int] = set()
    cfg.port = pick_port(cfg.port, socket.SOCK_STREAM, taken)
    cfg.ws_port = pick_port(cfg.ws_port, socket.SOCK_STREAM, taken)
    cfg.udp_port = pick_port(cfg.udp_port, socket.SOCK_DGRAM, taken)
    cfg.proxy_port = pick_port(cfg.proxy_port, socket.SOCK_DGRAM, taken)

    try:
        cfg.data_dir.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        print(f"sifted: cannot create the data folder {cfg.data_dir}: {e}", file=sys.stderr)
        return 1

    fusiond = find_fusiond(cfg.fusiond)
    app = App(cfg, fusiond)
    Handler.app = app
    httpd = http.server.ThreadingHTTPServer((HOST, cfg.port), Handler)
    httpd.daemon_threads = True
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    url = f"http://{HOST}:{cfg.port}/"
    bar = "-" * 60
    print(bar)
    print(f"  Sifted {VERSION} is running:  {url}")
    if fusiond is None:
        print("  WARNING: the fusion service is not built; run ./install.sh first.")
    print(f"  Recordings: {cfg.data_dir / 'runs'}")
    if cfg.background:
        print("  Running without a terminal; closing the app window stops Sifted.")
    else:
        print("  Keep this window open. Press Ctrl+C (or Quit in the app) to stop.")
    print(bar, flush=True)

    def on_signal(signum, _frame):
        app.quit.set()

    for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
        if sig is not None:
            signal.signal(sig, on_signal)

    if cfg.run:
        try:
            app.session.start_run(cfg.run, 1.0, "perfect", "none")
        except SessionError as e:
            print(f"sifted: {e}", file=sys.stderr)
    if not cfg.no_browser and (cfg.browser or not open_app_window(url)):
        open_browser(url)
    write_state(state_file, f"running {url}")
    if cfg.background:
        threading.Thread(target=app.watch_windows, daemon=True).start()

    while not app.quit.wait(0.5):
        pass
    print("\nStopping Sifted...", flush=True)
    app.session.shutdown()
    httpd.shutdown()
    try:
        state_file.unlink()
    except OSError:
        pass
    print("Stopped. Run ./sifted (or the Sifted shortcut) to start again.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
