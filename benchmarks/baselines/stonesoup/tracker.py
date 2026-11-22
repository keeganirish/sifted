"""A Stone Soup multi-target tracker configured like Sifted, writing a track report.

    python tracker.py <scenario_dir> <out.tracks>

Reads the scenario directory Sifted reads (sensors.cfg, frames.hex, truth.csv),
decodes the frames with sim/sifted_sim/wire.py (the repository's own Python
decoder), tracks with Stone Soup components, and writes the tracks it reports
at every truth instant in the track report format
(eval/sifted/eval/track_report.hpp), which sifted_montecarlo scores with
Sifted's own scorer. Truth positions are never read: truth.csv supplies only
the instants at which tracks are reported, as for Sifted.

Every mapping from Sifted's configuration, and every place where it cannot be
matched exactly, is in README.md ("Configuration mapping").
"""

from __future__ import annotations

import datetime as dt
import math
import pathlib
import sys
import time
from dataclasses import dataclass, field

import numpy as np
from ordered_set import OrderedSet
from scipy.stats import chi2
from stonesoup.base import Property
from stonesoup.dataassociator.neighbour import GNNWith2DAssignment
from stonesoup.deleter.time import UpdateTimeDeleter
from stonesoup.hypothesiser.distance import DistanceHypothesiser
from stonesoup.initiator.base import Initiator
from stonesoup.initiator.simple import MultiMeasurementInitiator, SimpleMeasurementInitiator
from stonesoup.measures import SquaredMahalanobis
from stonesoup.models.measurement.nonlinear import Cartesian2DToBearing, CartesianToBearingRange
from stonesoup.models.transition.linear import (CombinedLinearGaussianTransitionModel,
                                                ConstantVelocity)
from stonesoup.predictor.kalman import ExtendedKalmanPredictor
from stonesoup.types.angle import Bearing
from stonesoup.types.array import CovarianceMatrix, StateVector
from stonesoup.types.detection import Detection
from stonesoup.types.state import GaussianState
from stonesoup.types.update import Update
from stonesoup.updater.kalman import ExtendedKalmanUpdater

REPO = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "sim"))
from sifted_sim import sensors as sifted_sensors  # noqa: E402
from sifted_sim import wire  # noqa: E402


@dataclass(frozen=True)
class Config:
    """Sifted's defaults (src/sifted/core/engine.hpp, association/lifecycle.hpp)."""

    accel_psd: float = 4.0               # EngineConfig::accel_psd [m^2/s^3]
    initial_velocity_sigma: float = 100.0  # EngineConfig::initial_velocity_sigma [m/s]
    gate_probability: float = 0.999      # AssociationConfig::gate_probability
    confirm_hits: int = 3                # LifecycleConfig::confirm_hits
    tentative_timeout_s: float = 3.0     # LifecycleConfig::tentative_timeout_s
    coast_after_s: float = 2.5           # LifecycleConfig::coast_after_s
    drop_after_s: float = 10.0           # LifecycleConfig::drop_after_s
    max_position_sigma_m: float = 1000.0  # LifecycleConfig::max_position_sigma_m
    max_tracks: int = 1024               # EngineConfig::max_tracks (tentative included)


EPOCH = dt.datetime(1970, 1, 1)
STATE_ORDER = (0, 2, 1, 3)  # Stone Soup [x, vx, y, vy] -> report [x, y, vx, vy]
REPORT_HEADER = "sifted-tracks 1"
REPORT_COLUMNS = ("time_ns,track_id,status,last_update_ns,x_m,y_m,vx_mps,vy_mps,"
                  "p00,p01,p02,p03,p11,p12,p13,p22,p23,p33")


def to_datetime(ns: int) -> dt.datetime:
    if ns % 1000:
        raise ValueError(f"time {ns} ns is not a whole microsecond (datetime resolution)")
    return EPOCH + dt.timedelta(microseconds=ns // 1000)


class CappedInitiator(Initiator):
    """Starts tracks only while fewer than max_tracks are live (Sifted's TrackLimit)."""

    initiator: Initiator = Property(doc="what actually starts the tracks")
    free_slots: object = Property(doc="callable: how many more tracks may start")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.limited = 0

    def initiate(self, detections, timestamp, **kwargs):
        ordered = sorted(detections, key=lambda d: d.metadata["order"])
        room = max(0, self.free_slots())
        self.limited += max(0, len(ordered) - room)
        return self.initiator.initiate(OrderedSet(ordered[:room]), timestamp, **kwargs)


@dataclass
class TrackMeta:
    number: int                       # reported track id
    last_position: dt.datetime        # last range-bearing update
    coasting: bool = False
    first_order: tuple = field(default=())


def max_position_variance(cov: np.ndarray) -> float:
    block = cov[np.ix_((0, 2), (0, 2))]
    return float(np.linalg.eigvalsh(block)[-1])


def fmt(v: float) -> str:
    return repr(float(v))


@dataclass
class RunStats:
    seconds: float = 0.0
    track_limit: int = 0
    confirmed: int = 0


def track(scenario_dir: pathlib.Path, cfg: Config = Config()) -> tuple[str, RunStats]:
    table = {s.sensor_id: s for s in sifted_sensors.parse_table(
        (scenario_dir / "sensors.cfg").read_text(encoding="ascii"))}
    frames = [wire.decode(bytes.fromhex(line))
              for line in (scenario_dir / "frames.hex").read_text(encoding="ascii").split()]
    with (scenario_dir / "truth.csv").open(encoding="ascii") as f:
        next(f)
        instants = sorted({int(line.split(",", 1)[0]) for line in f if line.strip()})

    start = time.perf_counter()
    transition = CombinedLinearGaussianTransitionModel(
        [ConstantVelocity(cfg.accel_psd), ConstantVelocity(cfg.accel_psd)])
    predictor = ExtendedKalmanPredictor(transition)
    updater = ExtendedKalmanUpdater(measurement_model=None, use_joseph_cov=True)
    gates = {1: float(chi2.ppf(cfg.gate_probability, 1)), 2: float(chi2.ppf(cfg.gate_probability, 2))}
    associators = {dim: GNNWith2DAssignment(DistanceHypothesiser(
        predictor, updater, SquaredMahalanobis(), missed_distance=gate)) for dim, gate in gates.items()}
    models = {}
    for sid, s in table.items():
        offset = StateVector([s.x_m, s.y_m])
        rotation = StateVector([0.0, 0.0, s.yaw_rad])
        models[sid] = (s.kind, offset, rotation)

    model_cache: dict = {}

    def model_for(m: wire.Measurement):
        # One model object per (sensor, noise): Stone Soup caches measurement
        # predictions per (prediction, model), so sharing it avoids redoing
        # the Jacobian for every track-detection pair. Same numbers either way.
        key = (m.sensor_id, m.sigma)
        if key not in model_cache:
            model_cache[key] = make_model(m)
        return model_cache[key]

    def make_model(m: wire.Measurement):
        kind, offset, rotation = models[m.sensor_id]
        if kind != m.kind:
            raise ValueError(f"sensor {m.sensor_id}: frame kind {m.kind} != table kind {kind}")
        if kind == wire.RANGE_BEARING:
            return CartesianToBearingRange(
                ndim_state=4, mapping=(0, 2), noise_covar=CovarianceMatrix(
                    np.diag([m.sigma[1] ** 2, m.sigma[0] ** 2])),
                translation_offset=offset, rotation_offset=rotation)
        if kind == wire.BEARING_ONLY:
            return Cartesian2DToBearing(
                ndim_state=4, mapping=(0, 2), noise_covar=CovarianceMatrix([[m.sigma[0] ** 2]]),
                translation_offset=offset, rotation_offset=rotation)
        raise ValueError(f"sensor kind {kind} is not supported by the baseline")

    main: list = []                  # confirmed and coasting tracks, in id order
    meta: dict = {}
    stats = RunStats()
    v_var = cfg.initial_velocity_sigma ** 2
    prior = GaussianState(StateVector([0.0, 0.0, 0.0, 0.0]),
                          CovarianceMatrix(np.diag([0.0, v_var, 0.0, v_var])))
    holding_initiator = MultiMeasurementInitiator(
        prior_state=prior,
        deleter=UpdateTimeDeleter(dt.timedelta(seconds=cfg.tentative_timeout_s)),
        data_associator=associators[2], updater=updater, min_points=cfg.confirm_hits,
        initiator=CappedInitiator(
            initiator=SimpleMeasurementInitiator(prior_state=prior),
            free_slots=lambda: cfg.max_tracks - len(main) - len(holding_initiator.holding_tracks)))

    def maintain(t: dt.datetime) -> None:
        keep = []
        for trk in main:
            m = meta[trk]
            age = (t - m.last_position).total_seconds()
            drop = age > cfg.drop_after_s
            if not drop:
                pred = predictor.predict(trk.state, timestamp=t)
                drop = not max_position_variance(np.asarray(pred.covar)) <= cfg.max_position_sigma_m ** 2
            if drop:
                continue
            if not m.coasting and age > cfg.coast_after_s:
                m.coasting = True
            keep.append(trk)
        main[:] = keep

    groups: list[tuple[int, list[wire.Measurement]]] = []
    for m in frames:  # maximal runs of one sensor and one timestamp, like Engine::on_scan
        if groups and groups[-1][1][0].sensor_id == m.sensor_id and groups[-1][0] == m.time_ns:
            groups[-1][1].append(m)
        else:
            groups.append((m.time_ns, [m]))

    out = [REPORT_HEADER, REPORT_COLUMNS]
    next_group = 0
    next_number = 1
    order = 0
    for instant_ns in instants:
        while next_group < len(groups) and groups[next_group][0] <= instant_ns:
            t_ns, ms = groups[next_group]
            next_group += 1
            t = to_datetime(t_ns)
            maintain(t)
            dets = []
            for m in ms:
                if m.kind == wire.RANGE_BEARING:
                    vec = StateVector([Bearing(m.z[1]), m.z[0]])
                else:
                    vec = StateVector([Bearing(m.z[0])])
                dets.append(Detection(vec, timestamp=t, measurement_model=model_for(m),
                                      metadata={"order": (t_ns, m.sensor_id, order)}))
                order += 1
            kind = ms[0].kind
            associations = associators[2 if kind == wire.RANGE_BEARING else 1].associate(
                OrderedSet(main), OrderedSet(dets), t)
            used = set()
            for trk in main:
                hyp = associations.get(trk)
                if hyp is None:
                    continue
                if hyp:
                    trk.append(updater.update(hyp))
                    used.add(id(hyp.measurement))
                    if kind == wire.RANGE_BEARING:
                        meta[trk].last_position = t
                        meta[trk].coasting = False
                else:
                    trk.append(hyp.prediction)
            if kind != wire.RANGE_BEARING:
                continue  # bearings cannot start tracks (and are not given to tentative ones)
            left = OrderedSet(d for d in dets if id(d) not in used)
            confirmed = holding_initiator.initiate(left, t)
            for trk in sorted(confirmed, key=lambda x: x[0].hypothesis.measurement.metadata["order"]):
                meta[trk] = TrackMeta(next_number, t, False)
                next_number += 1
                main.append(trk)
            stats.confirmed = next_number - 1
        t_report = to_datetime(instant_ns)
        for trk in main:
            m = meta[trk]
            state = predictor.predict(trk.state, timestamp=t_report) \
                if trk.state.timestamp < t_report else trk.state
            x = np.asarray(state.state_vector, dtype=float).ravel()
            p = np.asarray(state.covar, dtype=float)
            last_update = next(s.timestamp for s in reversed(trk.states) if isinstance(s, Update))
            last_ns = int(round((last_update - EPOCH) / dt.timedelta(microseconds=1))) * 1000
            xs = [x[i] for i in STATE_ORDER]
            q = p[np.ix_(STATE_ORDER, STATE_ORDER)]
            q = np.tril(q) + np.tril(q, -1).T  # one triangle, mirrored: exactly symmetric
            cov = [q[a, b] for a in range(4) for b in range(a, 4)]
            out.append(f"{instant_ns},{m.number},{'coasting' if m.coasting else 'confirmed'},"
                       f"{last_ns}," + ",".join(fmt(v) for v in xs + cov))
    stats.seconds = time.perf_counter() - start
    stats.track_limit = holding_initiator.initiator.limited
    return "\n".join(out) + "\n", stats


def main(argv: list[str]) -> int:
    if len(argv) != 3:
        print(__doc__.splitlines()[2].strip(), file=sys.stderr)
        return 2
    text, stats = track(pathlib.Path(argv[1]))
    pathlib.Path(argv[2]).write_text(text, encoding="ascii", newline="\n")
    print(f"{stats.seconds:.3f} s, {stats.confirmed} tracks confirmed, "
          f"{stats.track_limit} track starts refused at the cap")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
