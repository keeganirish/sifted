"""ADS-B ingest: OpenSky Network state vectors -> wire-v1 GeoPosition frames over UDP.

    python3 -m sifted_sim.adsb --bbox 50.9,52.1,-1.0,1.0 --sensor-id 100 --target 127.0.0.1:5600
    python3 -m sifted_sim.adsb --from-file tests/samples/opensky_states_sample.json --print
    python3 -m sifted_sim.adsb --write-sensors adsb.cfg --sensor-id 100

The C++ core never parses HTTP or JSON: this adapter turns each OpenSky state
vector into one GeoPosition measurement (latitude, longitude, altitude) and
sends it like any other sensor. The daemon needs a sensor table with a
geo_position sensor of the same id, and an --origin near the area of interest
(the tangent-plane origin, ADR 0007).

Response format (OpenSky REST API, GET /api/states/all, as documented at
https://openskynetwork.github.io/opensky-api/rest.html): an object with
"time" (int, seconds) and "states" (list of state vectors or null). Each state
vector is a positional array; this adapter reads only these indices:

    0 icao24 (str)       3 time_position (int|null)   5 longitude (float|null)
    6 latitude (float|null)   7 baro_altitude (float|null, m)   8 on_ground (bool)
    13 geo_altitude (float|null, m)

All indices live in STATE_FIELDS below, so a format change is a one-line edit.

The response is untrusted (threat model B2). A state vector is dropped, and
the reason counted, if it is malformed, has no position or position time, is
on the ground (unless --include-ground), is older than --max-age seconds
relative to the response time, or repeats a position time already sent for
that aircraft. Survivors are sent in time order, and anything older than the
newest time already sent is dropped, because the engine refuses measurements
older than its clock. The C++ decoder validates every frame again.

Identity: the tracker associates ADS-B positions kinematically like any other
sensor; icao24 is used here only for per-aircraft duplicate suppression.

Rate limits and terms: OpenSky's documented quotas are 400 API credits per day
for anonymous users (10 s time resolution) and 4000 for registered users
(5 s), with a /states/all call costing 1-4 credits depending on the bounding
box area. The poll interval defaults to the smallest that stays within the
daily quota for the chosen box and is never allowed below the time
resolution. Authentication uses OAuth2 client credentials
(OPENSKY_CLIENT_ID / OPENSKY_CLIENT_SECRET environment variables). Use of
OpenSky data is subject to the OpenSky Network's terms of use.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from collections import Counter
from dataclasses import dataclass, field

from . import sensors, wire

API_URL = "https://opensky-network.org/api/states/all"
TOKEN_URL = "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token"

STATE_FIELDS = {"icao24": 0, "time_position": 3, "longitude": 5, "latitude": 6,
                "baro_altitude": 7, "on_ground": 8, "geo_altitude": 13}
MIN_STATE_LEN = max(STATE_FIELDS.values()) + 1
MAX_RESPONSE_BYTES = 32 * 1024 * 1024
MAX_STATES = 20000

ANON_CREDITS_PER_DAY, AUTH_CREDITS_PER_DAY = 400, 4000
ANON_RESOLUTION_S, AUTH_RESOLUTION_S = 10.0, 5.0
M_PER_DEG_LAT = 111_320.0
MIN_ALT_M, MAX_ALT_M = -1000.0, 100000.0  # same bounds as the C++ is_valid()


# Largest time_position whose nanosecond timestamp fits the wire format's
# int64 (found by tests/test_adsb_fuzz.py: a larger one made encode() raise).
MAX_TIME_POSITION_S = (2 ** 63 - 1) // 1_000_000_000


def credit_cost(lamin: float, lamax: float, lomin: float, lomax: float) -> int:
    """API credits per /states/all call for a bounding box (documented tiers)."""
    area = abs(lamax - lamin) * abs(lomax - lomin)
    return 1 if area <= 25 else 2 if area <= 100 else 3 if area <= 400 else 4


def min_interval_s(cost: int, authenticated: bool) -> float:
    credits = AUTH_CREDITS_PER_DAY if authenticated else ANON_CREDITS_PER_DAY
    resolution = AUTH_RESOLUTION_S if authenticated else ANON_RESOLUTION_S
    return max(resolution, 86400.0 * cost / credits)


def _is_num(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def _is_hex24(v) -> bool:
    return isinstance(v, str) and len(v) == 6 and all(c in "0123456789abcdef" for c in v.lower())


def geo_measurement(sensor_id: int, seq: int, time_ns: int, lat: float, lon: float, alt: float,
                    sigma_m: float = 50.0, sigma_alt_m: float = 50.0) -> wire.Measurement:
    """One ADS-B position as a GeoPosition measurement: a horizontal 1-sigma of
    sigma_m metres expressed in degrees of latitude and of longitude."""
    cos_lat = max(math.cos(math.radians(lat)), 0.01)
    return wire.Measurement(
        sensor_id=sensor_id, kind=wire.GEO_POSITION, seq=seq & 0xFFFFFFFF, time_ns=time_ns,
        z=(lat, lon, alt),
        sigma=(sigma_m / M_PER_DEG_LAT, sigma_m / (M_PER_DEG_LAT * cos_lat), sigma_alt_m))


@dataclass
class Adapter:
    """Converts responses to measurements, remembering what it already sent."""

    sensor_id: int
    sigma_m: float = 50.0       # assumed horizontal 1-sigma of an ADS-B position
    sigma_alt_m: float = 50.0
    max_age_s: float = 15.0
    include_ground: bool = False
    seq: int = 0
    newest_ns: int | None = None
    last_time: dict[str, int] = field(default_factory=dict)
    dropped: Counter = field(default_factory=Counter)

    def convert(self, response) -> list[wire.Measurement]:
        if not isinstance(response, dict) or not _is_num(response.get("time")):
            self.dropped["bad_response"] += 1
            return []
        states = response.get("states")
        if states is None:
            return []
        if not isinstance(states, list) or len(states) > MAX_STATES:
            self.dropped["bad_response"] += 1
            return []
        now = response["time"]
        candidates: list[tuple[int, str, float, float, float]] = []
        seen_now: set[str] = set()
        for row in states:
            reason = None
            if not isinstance(row, list) or len(row) < MIN_STATE_LEN:
                reason = "malformed"
            else:
                f = {k: row[i] for k, i in STATE_FIELDS.items()}
                icao, tpos, lat, lon = f["icao24"], f["time_position"], f["latitude"], f["longitude"]
                alt = f["geo_altitude"] if _is_num(f["geo_altitude"]) else f["baro_altitude"]
                if not _is_hex24(icao):
                    reason = "bad_icao24"
                elif tpos is None or lat is None or lon is None:
                    reason = "no_position"
                elif not (isinstance(tpos, int) and not isinstance(tpos, bool) and
                          0 < tpos <= MAX_TIME_POSITION_S and
                          _is_num(lat) and _is_num(lon) and -90 <= lat <= 90 and -180 <= lon <= 180):
                    reason = "bad_position"
                elif not isinstance(f["on_ground"], bool):
                    reason = "malformed"
                elif f["on_ground"] and not self.include_ground:
                    reason = "on_ground"
                elif not (_is_num(alt) and MIN_ALT_M <= alt <= MAX_ALT_M):
                    reason = "no_altitude"
                elif now - tpos > self.max_age_s or tpos > now + 5:
                    reason = "stale"
                elif tpos <= self.last_time.get(icao.lower(), -1) or icao.lower() in seen_now:
                    reason = "repeat"
                else:
                    seen_now.add(icao.lower())
                    candidates.append((tpos, icao.lower(), float(lat), float(lon), float(alt)))
            if reason:
                self.dropped[reason] += 1
        out: list[wire.Measurement] = []
        for tpos, icao, lat, lon, alt in sorted(candidates):
            t_ns = tpos * 1_000_000_000
            if self.newest_ns is not None and t_ns < self.newest_ns:
                self.dropped["older_than_sent"] += 1
                continue
            self.last_time[icao] = tpos
            self.newest_ns = t_ns
            out.append(geo_measurement(self.sensor_id, self.seq, t_ns, lat, lon, alt,
                                       self.sigma_m, self.sigma_alt_m))
            self.seq += 1
        return out


def fetch(bbox: tuple[float, float, float, float] | None, token: str | None, timeout: float = 20.0):
    query = {} if bbox is None else dict(zip(("lamin", "lamax", "lomin", "lomax"), bbox))
    url = API_URL + ("?" + urllib.parse.urlencode(query) if query else "")
    req = urllib.request.Request(url, headers={"Authorization": f"Bearer {token}"} if token else {})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        body = r.read(MAX_RESPONSE_BYTES + 1)
    if len(body) > MAX_RESPONSE_BYTES:
        raise ValueError("response too large")
    return json.loads(body)


def get_token(client_id: str, secret: str) -> str:
    data = urllib.parse.urlencode({"grant_type": "client_credentials", "client_id": client_id,
                                   "client_secret": secret}).encode()
    with urllib.request.urlopen(urllib.request.Request(TOKEN_URL, data=data), timeout=20) as r:
        return json.loads(r.read(1 << 16))["access_token"]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="sifted_sim.adsb", description=__doc__.splitlines()[0])
    ap.add_argument("--sensor-id", type=int, default=100)
    ap.add_argument("--target", default="127.0.0.1:5600")
    ap.add_argument("--bbox", help="lamin,lamax,lomin,lomax (strongly recommended: fewer credits)")
    ap.add_argument("--interval", type=float, help="poll interval [s]; default: the quota minimum")
    ap.add_argument("--from-file", help="convert a saved response instead of polling")
    ap.add_argument("--print", action="store_true", help="print frames as hex instead of sending")
    ap.add_argument("--sigma-m", type=float, default=50.0)
    ap.add_argument("--max-age", type=float, default=15.0)
    ap.add_argument("--include-ground", action="store_true")
    ap.add_argument("--write-sensors", help="write a sensor table for --sensor-id and exit")
    args = ap.parse_args(argv)

    if args.write_sensors:
        with open(args.write_sensors, "w", encoding="ascii", newline="\n") as f:
            f.write(sensors.format_table([sensors.SensorConfig(args.sensor_id, wire.GEO_POSITION, 0.0, 0.0, 0.0)]))
        return 0
    adapter = Adapter(args.sensor_id, args.sigma_m, max_age_s=args.max_age, include_ground=args.include_ground)
    host, _, port = args.target.rpartition(":")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def emit(ms: list[wire.Measurement]) -> None:
        for m in ms:
            frame = wire.encode(m)
            if args.print:
                print(frame.hex())
            else:
                sock.sendto(frame, (host, int(port)))
        print(f"adsb: {len(ms)} sent, dropped so far {dict(adapter.dropped)}", file=sys.stderr)

    if args.from_file:
        with open(args.from_file, "rb") as f:
            emit(adapter.convert(json.loads(f.read(MAX_RESPONSE_BYTES))))
        return 0

    bbox = tuple(float(v) for v in args.bbox.split(",")) if args.bbox else None
    if bbox is not None and len(bbox) != 4:
        ap.error("--bbox needs lamin,lamax,lomin,lomax")
    cid, secret = os.environ.get("OPENSKY_CLIENT_ID"), os.environ.get("OPENSKY_CLIENT_SECRET")
    authenticated = bool(cid and secret)
    floor = min_interval_s(credit_cost(*bbox) if bbox else 4, authenticated)
    interval = max(args.interval or floor, floor)
    if args.interval and args.interval < floor:
        print(f"adsb: --interval raised to {floor:.0f} s to stay within the OpenSky quota", file=sys.stderr)
    token, token_time, backoff = None, 0.0, interval
    while True:
        try:
            if authenticated and time.monotonic() - token_time > 25 * 60:  # tokens last 30 min
                token, token_time = get_token(cid, secret), time.monotonic()
            emit(adapter.convert(fetch(bbox, token)))
            backoff = interval
        except urllib.error.HTTPError as e:
            if e.code == 401:
                token_time = 0.0
            retry = e.headers.get("Retry-After") if e.headers else None
            backoff = float(retry) if retry and retry.isdigit() else min(backoff * 2, 3600.0)
            print(f"adsb: HTTP {e.code}; next attempt in {backoff:.0f} s", file=sys.stderr)
        except (OSError, ValueError) as e:
            backoff = min(backoff * 2, 3600.0)
            print(f"adsb: {e}; next attempt in {backoff:.0f} s", file=sys.stderr)
        time.sleep(backoff)


if __name__ == "__main__":
    sys.exit(main())
