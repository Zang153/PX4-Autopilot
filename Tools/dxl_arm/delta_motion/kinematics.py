"""Pure NumPy kinematics for the fixed-orientation, lower-assembly Delta.

SI units; frame fixed to the Delta base, +z up. The paired lower rods are
represented by one constant-length constraint per leg. This is a kinematic
model, not a collision certificate or a servo/dynamics model.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json

import numpy as np


@dataclass(frozen=True)
class DeltaParams:
    """Geometry defaults copied from airframes/delta_geometry.py:DeltaParams.

    Keep this small definition independent: importing the airframes package
    eagerly imports simulator-related modules. No XML or simulator is loaded.
    """

    base_radius: float = 0.18
    platform_radius: float = 0.07
    upper_arm: float = 0.18
    lower_arm: float = 0.34
    rod_spacing: float = 0.036
    home_angle_deg: float = 0.0
    joint_up_limit_deg: float = 30.0
    joint_down_limit_deg: float = 90.0


def _vectors(value: np.ndarray) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.ndim == 0 or result.shape[-1] != 3:
        raise ValueError("expected (..., 3) positions or joint angles")
    return result


class DeltaModel:
    """One continuous elbow-out IK branch and the lower three-sphere FK root.

    Public IK/FK return NaN for geometrically invalid OR safety-excluded states.
    q_min/q_max include joint_margin_rad. Singularities are screened using both
    a condition-number limit and normalized serial/parallel Jacobian margins;
    condition number alone does not exclude isotropically large amplification.
    """

    def __init__(self, params: DeltaParams = DeltaParams(), *,
                 joint_margin_rad: float = np.deg2rad(2.0),
                 max_condition: float = 30.0,
                 singular_margin: float = 0.03) -> None:
        # Also accept the original project's DeltaParams without importing it.
        self.params = DeltaParams(**{key: getattr(params, key) for key in asdict(DeltaParams())})
        values = np.array(list(asdict(self.params).values()), dtype=float)
        if not np.isfinite(values).all() or min(params.base_radius, params.platform_radius,
                                                params.upper_arm, params.lower_arm) <= 0:
            raise ValueError("geometry must be finite and lengths positive")
        if not np.isfinite(joint_margin_rad) or joint_margin_rad < 0:
            raise ValueError("joint margin must be finite and nonnegative")
        if not np.isfinite(max_condition) or max_condition < 1:
            raise ValueError("max_condition must be finite and at least one")
        if not np.isfinite(singular_margin) or not 0 < singular_margin < 1:
            raise ValueError("singular_margin must lie in (0, 1)")
        self.joint_margin_rad = float(joint_margin_rad)
        self.max_condition = float(max_condition)
        self.singular_margin = float(singular_margin)
        self.q_min = np.full(3, -np.deg2rad(params.joint_up_limit_deg) + joint_margin_rad)
        self.q_max = np.full(3, np.deg2rad(params.joint_down_limit_deg) - joint_margin_rad)
        if np.any(self.q_min >= self.q_max):
            raise ValueError("joint margin leaves no usable joint interval")
        angles = np.deg2rad([0., 120., 240.])
        self.radial = np.stack([np.cos(angles), np.sin(angles), np.zeros(3)], axis=-1)
        self.tangent = np.stack([-np.sin(angles), np.cos(angles), np.zeros(3)], axis=-1)
        self.length_tolerance = 1e-8 * (params.upper_arm + params.lower_arm)
        self.home = self.fk(np.full(3, np.deg2rad(params.home_angle_deg)))
        if not np.isfinite(self.home).all():
            raise ValueError("home is outside the selected safe assembly branch")

    def config(self) -> dict:
        return {"params": asdict(self.params), "joint_margin_rad": self.joint_margin_rad,
                "max_condition": self.max_condition, "singular_margin": self.singular_margin}

    @property
    def parameter_hash(self) -> str:
        return hashlib.sha256(json.dumps(self.config(), sort_keys=True).encode()).hexdigest()

    def elbows(self, q: np.ndarray) -> np.ndarray:
        q = _vectors(q)
        p = self.params
        with np.errstate(invalid="ignore"):
            elbow = (p.base_radius + p.upper_arm * np.cos(q))[..., :, None] * self.radial
            elbow[..., :, 2] = -p.upper_arm * np.sin(q)
        return elbow

    def _ik_raw(self, p: np.ndarray) -> np.ndarray:
        par = self.params
        relative = p[..., None, :] + (par.platform_radius - par.base_radius) * self.radial
        radial = np.sum(relative * self.radial, axis=-1)
        tangent = np.sum(relative * self.tangent, axis=-1)
        vertical = -relative[..., 2]
        distance = np.hypot(radial, vertical)
        with np.errstate(invalid="ignore", divide="ignore"):
            cosine = (par.upper_arm**2 + distance**2 - par.lower_arm**2 + tangent**2) / (2 * par.upper_arm * distance)
            q = np.arctan2(vertical, radial) - np.arccos(np.clip(cosine, -1, 1))
        valid = (np.isfinite(p).all(axis=-1) & (distance > 1e-12).all(axis=-1)
                 & (np.abs(tangent) <= par.lower_arm).all(axis=-1)
                 & (np.abs(cosine) <= 1 + 1e-12).all(axis=-1))
        return np.where(valid[..., None], q, np.nan)

    def _fk_raw(self, q: np.ndarray) -> np.ndarray:
        # Independent trilateration of three equal-radius spheres; no IK target
        # enters this calculation. Each center is elbow - platform anchor offset.
        centers = self.elbows(q) - self.params.platform_radius * self.radial
        a, b, c = centers[..., 0, :], centers[..., 1, :], centers[..., 2, :]
        ab, ac = b - a, c - a
        d = np.linalg.norm(ab, axis=-1)
        with np.errstate(invalid="ignore", divide="ignore"):
            ex = ab / d[..., None]
            i = np.sum(ac * ex, axis=-1)
            orthogonal = ac - i[..., None] * ex
            j = np.linalg.norm(orthogonal, axis=-1)
            ey = orthogonal / j[..., None]
            ez = np.cross(ex, ey)
            x = d / 2
            y = (np.sum(ac * ac, axis=-1) - 2 * i * x) / (2 * j)
            height_sq = self.params.lower_arm**2 - x**2 - y**2
            midpoint = a + x[..., None] * ex + y[..., None] * ey
            displacement = np.sqrt(np.maximum(height_sq, 0))[..., None] * ez
            lower = np.where((ez[..., 2] >= 0)[..., None], midpoint - displacement, midpoint + displacement)
        valid = (np.isfinite(q).all(axis=-1) & (d > 1e-12) & (j > 1e-12)
                 & (height_sq >= -1e-14) & (np.abs(ez[..., 2]) > 1e-8))
        return np.where(valid[..., None], lower, np.nan)

    def _jacobian(self, p: np.ndarray, q: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        par = self.params
        rods = p[..., None, :] + par.platform_radius * self.radial - self.elbows(q)
        with np.errstate(invalid="ignore"):
            derivative = (-par.upper_arm * np.sin(q))[..., :, None] * self.radial
            derivative[..., :, 2] = -par.upper_arm * np.cos(q)
        denominator = np.sum(rods * derivative, axis=-1)
        with np.errstate(invalid="ignore", divide="ignore"):
            inverse = rods / denominator[..., :, None]
        return inverse, rods, denominator / (par.upper_arm * par.lower_arm)

    def _valid_pair(self, p: np.ndarray, q: np.ndarray) -> np.ndarray:
        finite = np.isfinite(p).all(axis=-1) & np.isfinite(q).all(axis=-1)
        joint_ok = ((q >= self.q_min) & (q <= self.q_max)).all(axis=-1)
        branch_ok = ((np.abs(self._ik_raw(p) - q) < 1e-8).all(axis=-1)
                     & (np.linalg.norm(self._fk_raw(q) - p, axis=-1) < self.length_tolerance))
        inverse, rods, serial = self._jacobian(p, q)
        rod_ok = (np.abs(np.linalg.norm(rods, axis=-1) - self.params.lower_arm)
                  < self.length_tolerance).all(axis=-1)
        finite_matrix = np.isfinite(inverse).all(axis=(-2, -1))
        safe_inverse = np.where(finite_matrix[..., None, None], inverse, np.eye(3))
        singular_values = np.linalg.svd(safe_inverse, compute_uv=False)
        with np.errstate(divide="ignore", invalid="ignore"):
            condition = singular_values[..., 0] / singular_values[..., -1]
        safe_rods = np.where(finite[..., None, None], rods / self.params.lower_arm, np.eye(3))
        parallel_margin = np.linalg.svd(safe_rods, compute_uv=False)[..., -1]
        return (finite & joint_ok & branch_ok & rod_ok & finite_matrix
                & (condition <= self.max_condition)
                & (np.abs(serial) >= self.singular_margin).all(axis=-1)
                & (parallel_margin >= self.singular_margin))

    def ik(self, p: np.ndarray) -> np.ndarray:
        p = _vectors(p)
        q = self._ik_raw(p)
        return np.where(self._valid_pair(p, q)[..., None], q, np.nan)

    def fk(self, q: np.ndarray) -> np.ndarray:
        q = _vectors(q)
        p = self._fk_raw(q)
        return np.where(self._valid_pair(p, q)[..., None], p, np.nan)

    def valid_q(self, q: np.ndarray) -> np.ndarray:
        q = _vectors(q)
        return self._valid_pair(self._fk_raw(q), q)

    def valid_p(self, p: np.ndarray) -> np.ndarray:
        p = _vectors(p)
        return self._valid_pair(p, self._ik_raw(p))

    def inverse_jacobian(self, p: np.ndarray, q: np.ndarray | None = None) -> np.ndarray:
        p = _vectors(p)
        q = self._ik_raw(p) if q is None else _vectors(q)
        p, q = np.broadcast_arrays(p, q)
        inverse, _, _ = self._jacobian(p, q)
        return np.where(self._valid_pair(p, q)[..., None, None], inverse, np.nan)

    def rod_lengths(self, p: np.ndarray, q: np.ndarray) -> np.ndarray:
        p, q = np.broadcast_arrays(_vectors(p), _vectors(q))
        return np.linalg.norm(p[..., None, :] + self.params.platform_radius * self.radial
                              - self.elbows(q), axis=-1)
