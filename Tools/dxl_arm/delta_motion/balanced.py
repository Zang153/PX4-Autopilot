"""Episode-balanced random Delta motion, with a fixed mean-speed request.

An independently selected activity target is kept across candidate retries.
A legal fast random path is shortened to the requested arc length, then only
slowed down to fill the episode. Thus uniform time scaling never increases
the connector's joint velocity, acceleration or jerk. These are ideal joint
commands in the Delta-base frame, not an actuator-tracking certificate.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import time

import numpy as np

from .trajectory import Episode, MotionConfig, RandomMotion, Segment, episode_from_segments


ALGORITHM = "balanced_v2"
PROTOCOL = "episode_arc_length_retime_v2"


@dataclass(frozen=True)
class BalancedMotionConfig:
    duration: float = 5.0
    dt: float = 0.01
    geometry_dt: float = 0.005
    normalized_speed_bands: tuple = ((.08, .16), (.16, .24), (.24, .36),
                                      (.36, .50), (.50, .70))
    candidate_attempts: int = 4
    base_speed_range: tuple = (.4, 1.6)
    joint_speed_span_factor: float = 2.8
    joint_acceleration_span_factor: float = 20.0
    max_jerk: float = 400.0
    min_segment_time: float = .20
    target_attempts: int = 32
    arc_length_relative_tolerance: float = .001

    def __post_init__(self):
        numeric = (self.duration, self.dt, self.geometry_dt, self.joint_speed_span_factor,
                   self.joint_acceleration_span_factor, self.max_jerk, self.min_segment_time,
                   self.arc_length_relative_tolerance)
        if not np.isfinite(numeric).all() or min(numeric) <= 0:
            raise ValueError("Balanced motion times and limits must be finite and positive")
        if self.candidate_attempts < 1 or self.target_attempts < 1:
            raise ValueError("Balanced motion attempt budgets must be positive")
        bands = np.asarray(self.normalized_speed_bands, dtype=float)
        if (bands.ndim != 2 or bands.shape[1] != 2 or not len(bands)
                or not np.isfinite(bands).all() or np.any(bands[:, 0] <= 0)
                or np.any(bands[:, 1] <= bands[:, 0])
                or np.any(bands[1:, 0] < bands[:-1, 1])):
            raise ValueError("Activity bands must be finite, positive, ordered and non-overlapping")
        low, high = self.base_speed_range
        if not np.isfinite([low, high]).all() or low <= 0 or high <= low:
            raise ValueError("Invalid base speed range")
        steps = round(self.duration / self.dt)
        if steps < 1 or not np.isclose(steps * self.dt, self.duration, rtol=0, atol=1e-9):
            raise ValueError("Duration must be a positive integer multiple of dt")

    def motion_config(self, model) -> MotionConfig:
        ell = model.params.upper_arm + model.params.lower_arm
        span = np.deg2rad(model.params.joint_up_limit_deg + model.params.joint_down_limit_deg)
        return MotionConfig(
            duration=self.duration, dt=self.dt, geometry_dt=self.geometry_dt,
            max_qd=self.joint_speed_span_factor * span,
            max_qdd=self.joint_acceleration_span_factor * span, max_jerk=self.max_jerk,
            min_segment_time=self.min_segment_time, target_attempts=self.target_attempts,
            speed_ranges=(tuple(ell * value for value in self.base_speed_range),),
            speed_probabilities=(1.0,), local_radius_min=.08 * ell,
            local_radius_max=.35 * ell, stationary_probability=.02)


def _retimed_prefix(segments, cutoff: float, duration: float) -> list[Segment]:
    """Keep q(s) on a path prefix and uniformly dilate all its times."""
    if cutoff <= 0 or cutoff > duration + 1e-10:
        raise ValueError("Retiming may only slow a positive path prefix")
    scale = duration / cutoff
    result, remaining = [], cutoff
    for segment in segments:
        take = min(segment.duration, remaining)
        if take <= 1e-12:
            break
        fraction = take / segment.duration
        coefficients = segment.coefficients * fraction ** np.arange(6)[:, None]
        result.append(Segment(coefficients, take * scale))
        remaining -= take
        if remaining <= 1e-12:
            break
    if remaining > 1e-9:
        raise ValueError("Candidate path does not cover the requested prefix")
    return result


def _prefix_positions(model, segments, cutoff: float, steps: int) -> np.ndarray:
    """Sample a base-time prefix on the eventual command grid."""
    ends = np.cumsum([segment.duration for segment in segments])
    starts = np.r_[0., ends[:-1]]
    times = np.linspace(0., cutoff, steps + 1)
    indices = np.minimum(np.searchsorted(ends, times, side="right"), len(segments) - 1)
    q = np.empty((steps + 1, 3))
    for index, segment in enumerate(segments):
        mask = indices == index
        if mask.any():
            q[mask] = segment.evaluate(times[mask] - starts[index])
    # The final episode is checked using the public domain-aware FK and
    # Segment.check. This raw FK here only integrates an already checked path.
    return model._fk_raw(q)


def _path_length(positions) -> float:
    return float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())


class BalancedRandomMotion:
    def __init__(self, model, workspace, config: BalancedMotionConfig | None = None):
        self.model, self.workspace = model, workspace
        self.config = config or BalancedMotionConfig()
        self.motion_config = self.config.motion_config(model)
        self.arm_length = model.params.upper_arm + model.params.lower_arm

    def sample_activity(self, rng) -> dict:
        band = int(rng.integers(len(self.config.normalized_speed_bands)))
        lower, upper = self.config.normalized_speed_bands[band]
        target = float(rng.uniform(lower, upper))
        return {"speed_band": band, "target_normalized_speed": target,
                "target_mean_speed_m_s": target * self.arm_length,
                "normalized_speed_band": [float(lower), float(upper)]}

    def _activity(self, activity) -> dict:
        result = dict(activity)
        band = result.get("speed_band")
        if not isinstance(band, (int, np.integer)) or not 0 <= band < len(self.config.normalized_speed_bands):
            raise ValueError("Invalid episode activity band")
        target = float(result["target_normalized_speed"])
        lower, upper = self.config.normalized_speed_bands[band]
        if not np.isfinite(target) or not lower <= target <= upper:
            raise ValueError("Episode target is outside its fixed activity band")
        physical = target * self.arm_length
        if "target_mean_speed_m_s" in result and not np.isclose(
                result["target_mean_speed_m_s"], physical, rtol=1e-12, atol=1e-12):
            raise ValueError("Activity target uses a different geometry scale")
        return {"speed_band": int(band), "target_normalized_speed": target,
                "target_mean_speed_m_s": physical,
                "normalized_speed_band": [float(lower), float(upper)]}

    def generate(self, q0, rng, method="workspace", *, activity=None) -> Episode:
        started = time.perf_counter()
        config, motion = self.config, self.motion_config
        q0 = np.asarray(q0, dtype=float)
        if q0.shape != (3,) or not self.model.valid_q(q0):
            raise ValueError("Initial configuration is outside the feasible domain")
        activity = self._activity(self.sample_activity(rng) if activity is None else activity)
        target_length = activity["target_mean_speed_m_s"] * config.duration
        diagnostics = {"algorithm": ALGORITHM, "protocol": PROTOCOL, "method": method,
                       "activity": activity, "balanced_config": asdict(config),
                       "config": asdict(motion), "target_path_length_m": target_length,
                       "candidate_rejections": [], "constraint_violations": {}}
        steps = round(config.duration / config.dt)
        for attempt in range(config.candidate_attempts):
            # Slow requests do not need a full fast five-second path. Increase
            # only the construction horizon on retries; keep activity fixed.
            horizon = min(config.duration, max(1., activity["target_normalized_speed"]
                                               * config.duration / .6) * 1.4**attempt)
            base_config = replace(motion, duration=horizon)
            generator = RandomMotion(self.model, self.workspace, base_config)
            base = generator.generate(q0, rng, method=method)
            segments = [Segment(c, float(d)) for c, d in zip(
                base.coefficients, base.segment_durations, strict=True)]
            available = min(config.duration, float(base.t[-1])) if len(base.t) else 0.
            diagnostics["base_candidate_attempts"] = attempt + 1
            if not segments or available <= 0:
                diagnostics["candidate_rejections"].append({"reason": "empty_base_path",
                    "base_failure": base.diagnostics.get("failure_reason")})
                continue
            length = _path_length(_prefix_positions(self.model, segments, available, steps))
            if not np.isfinite(length) or length < target_length:
                diagnostics["candidate_rejections"].append({"reason": "insufficient_arc_length",
                    "available_length_m": length if np.isfinite(length) else None,
                    "available_duration_s": available, "base_success": base.success})
                continue
            lower, upper, cutoff = 0., available, available
            tolerance = config.arc_length_relative_tolerance * target_length
            # Use the actual 100 Hz chordal length, not endpoint displacement or
            # post-hoc quantiles. Bisection only selects an earlier base time.
            for _ in range(24):
                cutoff = .5 * (lower + upper)
                length = _path_length(_prefix_positions(self.model, segments, cutoff, steps))
                band_low, band_high = config.normalized_speed_bands[activity["speed_band"]]
                normalized_length = length / (self.arm_length * config.duration)
                if (abs(length - target_length) <= tolerance
                        and band_low <= normalized_length <= band_high):
                    break
                if length < target_length:
                    lower = cutoff
                else:
                    upper = cutoff
            retimed = _retimed_prefix(segments, cutoff, config.duration)
            reason = next((reason for segment in retimed
                           if (reason := segment.check(self.model, motion)) is not None), None)
            if reason is not None:
                diagnostics["candidate_rejections"].append({"reason": "retimed_" + reason})
                continue
            episode = episode_from_segments(self.model, retimed, motion, True, diagnostics)
            realized = _path_length(episode.p)
            normalized = realized / (self.arm_length * config.duration)
            low_band, high_band = config.normalized_speed_bands[activity["speed_band"]]
            if (not np.isfinite(episode.q).all() or not self.model.valid_q(episode.q).all()
                    or abs(realized - target_length) > tolerance * 1.01
                    or not low_band <= normalized <= high_band
                    or not self.model.valid_q(episode.q.astype(np.float32).astype(float)).all()):
                diagnostics["candidate_rejections"].append({"reason": "final_domain_or_activity_check"})
                continue
            diagnostics.update(realized_path_length_m=realized,
                               realized_mean_speed_m_s=realized / config.duration,
                               realized_normalized_speed=normalized,
                               base_duration_used_s=cutoff, time_dilation=config.duration / cutoff,
                               base_diagnostics=base.diagnostics, base_success=base.success,
                               segment_count=len(retimed), failure_reason=None,
                               generation_seconds=time.perf_counter() - started)
            return episode
        # A failed request never receives a slower activity label. Return a
        # legal zero-velocity initial state so the caller may retry or hold.
        zeros = np.zeros((1, 3))
        diagnostics.update(failure_reason="activity_budget_exhausted", segment_count=0,
                           realized_path_length_m=0., realized_mean_speed_m_s=0.,
                           realized_normalized_speed=0.,
                           generation_seconds=time.perf_counter() - started)
        return Episode(np.array([0.]), q0[None].copy(), zeros.copy(), zeros.copy(),
                       zeros.copy(), self.model.fk(q0)[None], zeros.copy(), zeros.copy(),
                       False, diagnostics, np.empty((0, 6, 3)), np.empty(0))
