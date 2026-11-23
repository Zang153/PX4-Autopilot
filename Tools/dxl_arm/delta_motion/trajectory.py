"""C2 random motion with a validated stopping continuation at every waypoint.

Limits are declared kinematic experiment limits, not hardware ratings. Geometry
is checked on a dense mesh; polynomial derivative extrema are checked exactly
up to floating point root finding. The mesh is not a continuous collision proof.
"""
from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

import numpy as np

from .workspace import WorkspaceSamplingError


@dataclass(frozen=True)
class MotionConfig:
    duration: float = 5.0
    dt: float = 0.01
    geometry_dt: float = 0.01
    max_qd: float = 3.0
    max_qdd: float = 20.0
    max_jerk: float = 200.0
    min_segment_time: float = 0.35
    max_segment_time: float = 2.5
    target_attempts: int = 24
    global_probability: float = 0.65
    local_radius_min: float = 0.04
    local_radius_max: float = 0.18
    speed_ranges: tuple = ((0.06, 0.16), (0.16, 0.35), (0.35, 0.65))
    speed_probabilities: tuple = (0.15, 0.45, 0.40)
    through_velocity_ratio: tuple = (0.2, 0.75)
    stationary_probability: float = 0.04

    def __post_init__(self):
        if min(self.duration, self.dt, self.geometry_dt, self.max_qd,
               self.max_qdd, self.max_jerk, self.min_segment_time) <= 0:
            raise ValueError("Times and limits must be positive")
        if self.target_attempts < 1 or self.max_segment_time < self.min_segment_time:
            raise ValueError("Invalid candidate budget or segment times")
        if not 0 <= self.global_probability <= 1:
            raise ValueError("Invalid global probability")
        if not np.isclose(sum(self.speed_probabilities), 1):
            raise ValueError("Speed probabilities must sum to one")


@dataclass
class Segment:
    coefficients: np.ndarray  # normalized time, ascending powers, (6,3)
    duration: float

    @classmethod
    def connect(cls, q0, v0, a0, q1, v1, a1, duration):
        if duration <= 0:
            raise ValueError("Segment duration must be positive")
        c = np.zeros((6, 3), dtype=float)
        c[0], c[1], c[2] = q0, np.asarray(v0) * duration, np.asarray(a0) * duration**2 / 2
        d = np.asarray(q1) - c[0] - c[1] - c[2]
        e = np.asarray(v1) * duration - c[1] - 2 * c[2]
        f = np.asarray(a1) * duration**2 - 2 * c[2]
        c[3], c[4], c[5] = 10*d-4*e+f/2, -15*d+7*e-f, 6*d-3*e+f/2
        return cls(c, float(duration))

    def derivative_coefficients(self, order):
        c = self.coefficients.copy()
        for _ in range(order):
            c = c[1:] * np.arange(1, len(c))[:, None] / self.duration
        return c

    def evaluate(self, times, order=0):
        s = np.asarray(times, dtype=float) / self.duration
        c = self.derivative_coefficients(order)
        return np.polynomial.polynomial.polyval(s, c).T

    def extrema(self, order):
        c = self.derivative_coefficients(order)
        bounds = np.empty((2, 3))
        for axis in range(3):
            derivative = c[1:, axis] * np.arange(1, len(c))
            roots = np.polynomial.polynomial.polyroots(derivative)
            roots = roots.real[(np.abs(roots.imag) < 1e-9) & (roots.real > 0) & (roots.real < 1)]
            values = np.polynomial.polynomial.polyval(np.r_[0., roots, 1.], c[:, axis])
            bounds[:, axis] = values.min(), values.max()
        return bounds

    def check(self, model, config):
        bounds = self.extrema(0)
        if np.any(bounds[0] < model.q_min - 1e-10) or np.any(bounds[1] > model.q_max + 1e-10):
            return "joint_limit"
        for order, limit, name in ((1, config.max_qd, "velocity"), (2, config.max_qdd, "acceleration"),
                                   (3, config.max_jerk, "jerk")):
            if np.max(np.abs(self.extrema(order))) > limit + 1e-9:
                return name
        t = np.linspace(0, self.duration, int(np.ceil(self.duration/config.geometry_dt))+1)
        if not np.all(model.valid_q(self.evaluate(t))):
            return "geometry_or_branch"
        return None


@dataclass
class Episode:
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray
    jerk: np.ndarray
    p: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    success: bool
    diagnostics: dict
    coefficients: np.ndarray
    segment_durations: np.ndarray

    @classmethod
    def empty_failure(cls, diagnostics):
        """An attempted reset with no sampled state contributes zero exposure."""
        return cls(np.empty(0), *(np.empty((0, 3)) for _ in range(7)),
                   False, diagnostics, np.empty((0, 6, 3)), np.empty(0))

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, **{k: getattr(self, k) for k in (
            "t", "q", "qd", "qdd", "jerk", "p", "velocity", "acceleration",
            "coefficients", "segment_durations")}, success=self.success,
            diagnostics=json.dumps(self.diagnostics, sort_keys=True))

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            values = {k: data[k] for k in ("t", "q", "qd", "qdd", "jerk", "p", "velocity",
                                          "acceleration", "coefficients", "segment_durations")}
            return cls(**values, success=bool(data["success"]), diagnostics=json.loads(str(data["diagnostics"])))


def sample_segments(segments, duration, dt):
    ends = np.cumsum([s.duration for s in segments])
    available = min(duration, float(ends[-1]))
    t = np.arange(int(np.floor(available/dt))+1) * dt
    if available - t[-1] > 1e-10:
        t = np.r_[t, available]
    else:
        t[-1] = available
    indices = np.minimum(np.searchsorted(ends, t, side="right"), len(segments)-1)
    starts = np.r_[0., ends[:-1]]
    values = [np.zeros((len(t), 3)) for _ in range(4)]
    for i, segment in enumerate(segments):
        selected = indices == i
        for order in range(4):
            values[order][selected] = segment.evaluate(t[selected] - starts[i], order)
    return t, values


def episode_from_segments(model, segments, config, success, diagnostics):
    t, (q, qd, qdd, jerk) = sample_segments(segments, config.duration, config.dt)
    p = model.fk(q)
    jinv = model.inverse_jacobian(p, q)
    velocity = np.linalg.solve(jinv, qd[..., None])[..., 0]
    # Analytic Jdot term for sphere constraints: l.(p'-e')=0.
    radial = model.radial
    upper = model.params.upper_arm
    e = model.elbows(q)
    anchors = p[:, None, :] + model.params.platform_radius * radial
    lower = anchors - e
    e_q = -upper*np.sin(q)[..., None]*radial
    e_q[..., 2] = -upper*np.cos(q)
    e_qq = -upper*np.cos(q)[..., None]*radial
    e_qq[..., 2] = upper*np.sin(q)
    e_dot = e_q * qd[..., None]
    e_ddot = e_q*qdd[..., None] + e_qq*qd[..., None]**2
    rhs = np.sum(lower*e_ddot, axis=-1) - np.sum((velocity[:, None, :]-e_dot)**2, axis=-1)
    acceleration = np.linalg.solve(lower, rhs[..., None])[..., 0]
    return Episode(t, q, qd, qdd, jerk, p, velocity, acceleration, success, diagnostics,
                   np.asarray([s.coefficients for s in segments]),
                   np.asarray([s.duration for s in segments]))


class RandomMotion:
    """Identical connector/limits for joint-target baseline and task-space target.

    Each accepted segment has a checked stop continuation. When proposals fail,
    that continuation is used without discontinuities; a stopped state that
    cannot restart is reported as a failed partial episode.
    """

    def __init__(self, model, workspace, config=None):
        self.model, self.workspace = model, workspace
        self.config = config or MotionConfig()

    def _target(self, q, rng, method):
        cfg, model = self.config, self.model
        local = rng.random() > cfg.global_probability
        if method == "joint":
            candidate = (q + rng.normal(size=3) * rng.uniform(.15, .60)
                         if local else rng.uniform(model.q_min, model.q_max))
            return candidate if model.valid_q(candidate) else None
        if method != "workspace":
            raise ValueError(f"Unknown method {method!r}")
        if local:
            direction = rng.normal(size=3)
            direction /= np.linalg.norm(direction)
            p = model.fk(q) + direction*rng.uniform(cfg.local_radius_min, cfg.local_radius_max)
            candidate = model.ik(p)
            return candidate if model.valid_q(candidate) else None
        _, qs = self.workspace.sample(rng, 1)
        return qs[0]

    def _brake(self, q, v):
        cfg = self.config
        duration = max(.25, 2*np.max(np.abs(v))/cfg.max_qdd,
                       np.sqrt(8*np.max(np.abs(v))/cfg.max_jerk))
        return Segment.connect(q, v, np.zeros(3), q + .5*duration*v,
                               np.zeros(3), np.zeros(3), duration)

    def generate(self, q0, rng, method="workspace"):
        started = time.perf_counter()
        cfg, model = self.config, self.model
        q, v, a = np.asarray(q0, dtype=float).copy(), np.zeros(3), np.zeros(3)
        if not model.valid_q(q):
            raise ValueError("Initial configuration is not in the declared feasible domain")
        elapsed, segments, backup = 0., [], None
        attempts, rejections, brake_count = 0, Counter(), 0
        requested_speeds, accepted_speeds = [], []
        failures = None
        while elapsed < cfg.duration - 1e-12:
            accepted = None
            for _ in range(cfg.target_attempts):
                attempts += 1
                try:
                    target = self._target(q, rng, method)
                except WorkspaceSamplingError:
                    rejections["target_sampling_exhausted"] += 1
                    continue
                if target is None:
                    rejections["target_infeasible"] += 1
                    continue
                speed_bin = rng.choice(len(cfg.speed_ranges), p=cfg.speed_probabilities)
                speed = rng.uniform(*cfg.speed_ranges[speed_bin])
                requested_speeds.append(float(speed))
                distance = np.linalg.norm(model.fk(target) - model.fk(q))
                duration = distance/speed * rng.uniform(.85, 1.15)
                if duration > cfg.max_segment_time:
                    rejections["distance_time_budget"] += 1
                    continue
                duration = max(cfg.min_segment_time, duration)
                # The endpoint follows the arrival direction. A random sideways
                # component changes the continuation; a stopping tail certifies it.
                direction = target - q
                side = rng.normal(size=3)
                side *= .15*np.linalg.norm(direction)/max(np.linalg.norm(side), 1e-12)
                v1 = rng.uniform(*cfg.through_velocity_ratio)*(direction+side)/duration
                if rng.random() < cfg.stationary_probability:
                    v1[:] = 0
                segment = Segment.connect(q, v, a, target, v1, np.zeros(3), duration)
                reason = segment.check(model, cfg)
                if reason is not None:
                    rejections[reason] += 1
                    continue
                stop = self._brake(target, v1)
                reason = stop.check(model, cfg)
                if reason is not None:
                    rejections["brake_"+reason] += 1
                    continue
                accepted = (segment, stop, target, v1)
                accepted_speeds.append(float(speed))
                break
            if accepted is None:
                if backup is not None:
                    segments.append(backup)
                    elapsed += backup.duration
                    q, v, a = (backup.evaluate(backup.duration, order) for order in range(3))
                    backup = None
                    brake_count += 1
                    continue
                failures = "no_feasible_continuation"
                break
            segment, backup, q, v = accepted
            segments.append(segment)
            a = np.zeros(3)
            elapsed += segment.duration
        success = elapsed >= cfg.duration - 1e-10
        diagnostics = {
            "method": method, "candidate_attempts": attempts,
            "rejections": dict(rejections), "brake_fallbacks": brake_count,
            "failure_reason": failures, "constraint_violations": {},
            "requested_speeds": requested_speeds, "accepted_requested_speeds": accepted_speeds,
            "segment_count": len(segments), "config": asdict(cfg),
        }
        if not segments:
            # Only the actual initial state was exposed; no fabricated dwell.
            zero = np.zeros((1, 3))
            episode = Episode(np.array([0.]), q[None, :], zero.copy(), zero.copy(),
                              zero.copy(), model.fk(q)[None, :], zero.copy(), zero.copy(),
                              False, diagnostics, np.empty((0, 6, 3)), np.empty(0))
        else:
            episode = episode_from_segments(model, segments, cfg, success, diagnostics)
        if not np.all(model.valid_q(episode.q)):
            episode.success = False
            diagnostics["constraint_violations"]["dense_geometry"] = 1
        diagnostics["generation_seconds"] = time.perf_counter() - started
        return episode
