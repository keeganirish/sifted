"""WGS-84 geodetic <-> local East-North-Up tangent plane (Python mirror).

Same formulas as the C++ core (src/sifted/estimation/geodetic.cpp, ADR 0007):
geodetic -> ECEF -> rotate into the ENU basis at a fixed origin. The real-data
evaluation (sifted_sim.realeval) uses this to place recorded flight paths in
a local frame for the simulated sensors (Evaluation B) and to measure
distances while cleaning. Evaluation A never uses it for scoring: the C++
tool projects with the engine's own code.

Distances go through ECEF, so they are correct across the dateline and near
the poles, where naive latitude/longitude differences are not.
"""

from __future__ import annotations

import math

A_M = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)


def to_ecef(lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> tuple[float, float, float]:
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    n = A_M / math.sqrt(1.0 - E2 * sl * sl)
    return ((n + alt_m) * cl * math.cos(lon), (n + alt_m) * cl * math.sin(lon),
            (n * (1.0 - E2) + alt_m) * sl)


def chord_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Straight-line distance between two points on the ellipsoid surface [m].

    Below a few hundred kilometres it differs from the geodesic by less than
    the noise of any position report (r^3 / 24 R^2: 0.6 m at 100 km).
    """
    a, b = to_ecef(lat1, lon1), to_ecef(lat2, lon2)
    return math.dist(a, b)


class Enu:
    """The ENU tangent plane at a fixed origin."""

    def __init__(self, lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> None:
        if not (-90.0 <= lat_deg <= 90.0 and -180.0 <= lon_deg <= 180.0):
            raise ValueError("origin out of range")
        self.lat_deg, self.lon_deg, self.alt_m = lat_deg, lon_deg, alt_m
        self._o = to_ecef(lat_deg, lon_deg, alt_m)
        lat, lon = math.radians(lat_deg), math.radians(lon_deg)
        sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
        self._east = (-so, co, 0.0)
        self._north = (-sl * co, -sl * so, cl)

    def project(self, lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> tuple[float, float]:
        """(east, north) of a geodetic point [m]; "up" is dropped, as the tracker does."""
        p = to_ecef(lat_deg, lon_deg, alt_m)
        d = (p[0] - self._o[0], p[1] - self._o[1], p[2] - self._o[2])
        return (sum(e * x for e, x in zip(self._east, d)),
                sum(n * x for n, x in zip(self._north, d)))

    def velocity(self, lat_deg: float, lon_deg: float, speed_mps: float,
                 track_deg: float) -> tuple[float, float]:
        """A horizontal velocity reported at a point (ground speed, true track
        clockwise from north) as (v_east, v_north) in this plane.

        Rotated through ECEF rather than used as is, so meridian convergence
        between the report's position and the origin is accounted for.
        """
        lat, lon = math.radians(lat_deg), math.radians(lon_deg)
        tr = math.radians(track_deg)
        ve, vn = speed_mps * math.sin(tr), speed_mps * math.cos(tr)
        sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
        v = tuple(ve * e + vn * n for e, n in zip((-so, co, 0.0), (-sl * co, -sl * so, cl)))
        return (sum(e * x for e, x in zip(self._east, v)),
                sum(n * x for n, x in zip(self._north, v)))
