"""Streaming reader for OpenSky historical state-vector CSV (untrusted input).

    python3 -m sifted_sim.statevectors FILE [FILE ...]     # counts per file

OpenSky publishes historical state vectors as CSV with a header row: the
`state_vectors_data4` table of its Trino database, which is also the format
of its former public dumps (states_YYYY-MM-DD-HH.csv) and of the incident
extracts in its data-samples bucket. Columns are found by name, so extra
columns (time_utc, serials, *_ft, position_age_s, ...) and reordering are
tolerated. Required: time, icao24, lat, lon, onground, lastposupdate.
Optional: velocity, heading, vertrate, baroaltitude, geoaltitude.

Semantics (OpenSky documentation): `time` is when the state vector was
sampled (integer seconds), `lastposupdate` when its position was last
updated (seconds, millisecond decimals), `velocity` ground speed [m/s],
`heading` true track [deg clockwise from north], altitudes in metres. A state
vector repeats its last position until a new position message arrives, so
the position time is lastposupdate, not time.

Threat model B6 (docs/threat-model.md): the file is untrusted. It is read as
a stream, never wholly into memory; files over MAX_FILE_BYTES, lines over
MAX_LINE_BYTES and more than MAX_ROWS rows are refused, and an archive is
read member by member without extracting anything to disk. Every row is
validated field by field; a bad row is dropped and the reason counted, never
raised. Checks match the live adapter (sifted_sim.adsb) where they overlap:
the same icao24, latitude/longitude, altitude bounds, 15 s maximum position
age and 5 s future tolerance.
"""

from __future__ import annotations

import csv
import io
import math
import pathlib
import re
import sys
import tarfile
from collections import Counter
from dataclasses import dataclass
from typing import BinaryIO, Iterator

from .adsb import MAX_ALT_M, MAX_TIME_POSITION_S, MIN_ALT_M, _is_hex24

MAX_FILE_BYTES = 1 << 30      # 1 GiB of CSV per file or archive member
MAX_LINE_BYTES = 1 << 16      # a state vector row is ~300 bytes
MAX_ROWS = 20_000_000
MAX_POSITION_AGE_S = 15.0     # time - lastposupdate; as adsb.Adapter.max_age_s
FUTURE_TOLERANCE_S = 5.0      # lastposupdate - time; as adsb.Adapter
MEMBER_PATTERN = re.compile(r"(^|/)[^/]*state_vectors[^/]*\.csv$")

REQUIRED = ("time", "icao24", "lat", "lon", "onground", "lastposupdate")
OPTIONAL = ("velocity", "heading", "vertrate", "baroaltitude", "geoaltitude")
_SECONDS = re.compile(r"^([0-9]{1,12})(?:\.([0-9]{1,9}))?$")
_TRUE, _FALSE = {"true", "1", "t"}, {"false", "0", "f"}


class FormatError(ValueError):
    """The file as a whole cannot be read (no header, too large, truncated archive)."""


@dataclass(frozen=True)
class Report:
    """One validated position report (a state vector whose position is usable)."""

    icao24: str          # lower-case hex; identity, used for scoring only
    t_ms: int            # position time (lastposupdate), integer milliseconds
    row_time_s: int      # state vector time
    lat: float
    lon: float
    alt_m: float | None  # geometric altitude, else barometric (None only on the ground)
    speed_mps: float | None
    track_deg: float | None
    vrate_mps: float | None
    on_ground: bool


def parse_seconds_ms(s: str) -> int | None:
    """'1792331998.998' -> 1792331998998, exactly (no binary floating point).

    Decimals beyond milliseconds are rounded half up. None if not a plain
    non-negative decimal.
    """
    m = _SECONDS.match(s)
    if m is None:
        return None
    frac = (m.group(2) or "").ljust(4, "0")
    ms = int(m.group(1)) * 1000 + int(frac[:3])
    return ms + (1 if int(frac[3]) >= 5 else 0)


def _num(s: str) -> float | None:
    if s == "":
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def _bool(s: str) -> bool | None:
    v = s.strip().lower()
    return True if v in _TRUE else False if v in _FALSE else None


class Reader:
    """Validates rows of one or more files, counting what it drops and why."""

    def __init__(self) -> None:
        self.dropped: Counter = Counter()
        self.rows = 0  # data rows seen (valid or not)

    def _row(self, f: list[str], col: dict[str, int]) -> Report | None:
        g = {k: f[i].strip() for k, i in col.items()}
        icao = g["icao24"].lower()
        if not _is_hex24(icao):
            return self._drop("bad_icao24")
        if g["lat"] == "" or g["lon"] == "" or g["lastposupdate"] == "":
            return self._drop("no_position")
        lat, lon = _num(g["lat"]), _num(g["lon"])
        if lat is None or lon is None or not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
            return self._drop("bad_position")
        t_ms = parse_seconds_ms(g["lastposupdate"])
        row_ms = parse_seconds_ms(g["time"])
        if (t_ms is None or row_ms is None or not 0 < t_ms <= MAX_TIME_POSITION_S * 1000
                or not 0 < row_ms <= MAX_TIME_POSITION_S * 1000):
            return self._drop("bad_time")
        if t_ms - row_ms > FUTURE_TOLERANCE_S * 1000:
            return self._drop("future")
        if row_ms - t_ms > MAX_POSITION_AGE_S * 1000:
            return self._drop("stale")
        on_ground = _bool(g["onground"])
        if on_ground is None:
            return self._drop("malformed")
        alt = _num(g.get("geoaltitude", ""))
        if alt is None or not MIN_ALT_M <= alt <= MAX_ALT_M:
            alt = _num(g.get("baroaltitude", ""))
        if alt is None or not MIN_ALT_M <= alt <= MAX_ALT_M:
            if not on_ground:  # surface reports often carry no altitude
                return self._drop("no_altitude")
            alt = None
        speed, track = _num(g.get("velocity", "")), _num(g.get("heading", ""))
        if speed is None or track is None or not (0.0 <= speed <= 1000.0 and -360.0 <= track <= 720.0):
            speed, track = None, None  # velocity is optional; a bad one is not used
        else:
            track = math.fmod(track + 360.0, 360.0)
        vrate = _num(g.get("vertrate", ""))
        return Report(icao, t_ms, row_ms // 1000, lat, lon, alt, speed, track,
                      vrate if vrate is not None and abs(vrate) <= 500.0 else None, on_ground)

    def _drop(self, reason: str) -> None:
        self.dropped[reason] += 1
        return None

    def read_stream(self, stream: BinaryIO) -> Iterator[Report]:
        """Rows of one CSV byte stream, validated. Raises FormatError for a bad file."""
        total = 0

        def lines() -> Iterator[bytes]:
            nonlocal total
            while True:
                line = stream.readline(MAX_LINE_BYTES + 1)
                if not line:
                    return
                total += len(line)
                if total > MAX_FILE_BYTES:
                    raise FormatError(f"file larger than {MAX_FILE_BYTES} bytes")
                yield line

        it = lines()
        try:
            header_line = next(it)
        except StopIteration:
            raise FormatError("empty file (no header)") from None
        if len(header_line) > MAX_LINE_BYTES:
            raise FormatError("header line too long")
        header = next(csv.reader([header_line.decode("ascii", "replace")]))
        names = [h.strip().lower() for h in header]
        col = {}
        for k in REQUIRED + OPTIONAL:
            if names.count(k) > 1:
                raise FormatError(f"duplicate column {k!r}")
            if k in names:
                col[k] = names.index(k)
        missing = [k for k in REQUIRED if k not in col]
        if missing:
            raise FormatError(f"not an OpenSky state-vector CSV: missing columns {missing}")
        width = len(names)
        for raw in it:
            self.rows += 1
            if self.rows > MAX_ROWS:
                raise FormatError(f"more than {MAX_ROWS} rows")
            if len(raw) > MAX_LINE_BYTES or not raw.endswith(b"\n"):
                # Over-long, or the unterminated last line of a truncated file.
                self._drop("malformed")
                if len(raw) > MAX_LINE_BYTES:  # skip the rest of the long line
                    while raw and not raw.endswith(b"\n"):
                        raw = stream.readline(MAX_LINE_BYTES + 1)
                        total += len(raw)
                continue
            try:
                text = raw.decode("ascii")
                fields = next(csv.reader([text]))
            except (UnicodeDecodeError, csv.Error, StopIteration):
                self._drop("malformed")
                continue
            if len(fields) != width:
                self._drop("malformed")
                continue
            r = self._row(fields, col)
            if r is not None:
                yield r

    def read_path(self, path: str) -> Iterator[tuple[str, Report]]:
        """(member name, report) for a .csv file, or for every state-vector CSV
        member of a .tar/.tar.gz archive (read in archive order, not extracted)."""
        if path.endswith((".tar", ".tar.gz", ".tgz")):
            found = 0
            try:
                with tarfile.open(path, "r:*") as tar:
                    for member in tar:
                        if not member.isfile() or not MEMBER_PATTERN.search(member.name):
                            continue
                        parts = pathlib.PurePosixPath(member.name).parts
                        if member.name.startswith("/") or ".." in parts:
                            continue  # never extracted anyway; refuse odd names outright
                        if member.size > MAX_FILE_BYTES:
                            raise FormatError(f"{member.name}: larger than {MAX_FILE_BYTES} bytes")
                        found += 1
                        f = tar.extractfile(member)
                        assert f is not None
                        for r in self.read_stream(f):
                            yield member.name, r
            except (tarfile.TarError, EOFError, OSError) as e:
                raise FormatError(f"{path}: unreadable or truncated archive: {e}") from e
            if found == 0:
                raise FormatError(f"{path}: no *state_vectors*.csv member")
            return
        with open(path, "rb") as f:
            for r in self.read_stream(f):
                yield path, r


def read_bytes(data: bytes) -> tuple[list[Report], Reader]:
    """Convenience for tests: every report of an in-memory CSV."""
    rd = Reader()
    return list(rd.read_stream(io.BytesIO(data))), rd


def main(argv: list[str] | None = None) -> int:
    args = sys.argv[1:] if argv is None else argv
    if not args:
        print(__doc__.splitlines()[2].strip(), file=sys.stderr)
        return 2
    for path in args:
        rd = Reader()
        n = sum(1 for _ in rd.read_path(path))
        print(f"{path}: {rd.rows} rows, {n} valid, dropped {dict(sorted(rd.dropped.items()))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
