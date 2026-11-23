"""Trajectory-independent Cartesian voxel reference for the ideal Delta model.

A marked voxel has at least one certified kinematic sample, not necessarily an
entirely feasible interior. Six-neighbor connectivity is a grid approximation;
it does not certify that every pair of representatives can be joined directly.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import hashlib
from pathlib import Path

import numpy as np

from .kinematics import DeltaModel, DeltaParams, _vectors


class WorkspaceSamplingError(RuntimeError):
    """The bounded continuous sampler could not find a valid point."""


@dataclass
class Workspace:
    points: np.ndarray
    joints: np.ndarray
    origin: np.ndarray
    spacing: float
    shape: tuple[int, int, int]
    mask: np.ndarray
    grid_ids: np.ndarray
    metadata: dict
    model: DeltaModel

    def cell_ids(self, p: np.ndarray) -> np.ndarray:
        """Map positions to marked voxels; this lookup is not a point IK check."""
        p = _vectors(p)
        finite = np.isfinite(p).all(axis=-1)
        safe_p = np.where(finite[..., None], p, self.origin)
        coordinates = np.floor((safe_p - self.origin) / self.spacing).astype(np.int64)
        inside = finite & ((coordinates >= 0) & (coordinates < self.shape)).all(axis=-1)
        coordinates = np.clip(coordinates, 0, np.asarray(self.shape) - 1)
        flat = ((coordinates[..., 0] * self.shape[1] + coordinates[..., 1])
                * self.shape[2] + coordinates[..., 2])
        return np.where(inside & self.mask.ravel()[flat], flat, -1)

    def sample(self, rng: np.random.Generator, n: int) -> tuple[np.ndarray, np.ndarray]:
        """Choose voxels uniformly, then continuously sample legal positions.

        The first 64 rejection rounds are uniform within the selected voxel.
        Thin boundary cells use shrinking continuous jitter around their known
        valid representative; this fallback is not uniform over feasible volume.
        It preserves the selected-cell distribution and never silently switches
        a difficult cell to an easy one. Exhaustion raises an explicit error.
        """
        if n < 0:
            raise ValueError("n must be nonnegative")
        chosen = rng.integers(0, len(self.points), size=n)
        ids = self.grid_ids[chosen]
        coordinates = np.stack(np.unravel_index(ids, self.shape), axis=-1)
        lower = self.origin + self.spacing * coordinates
        result_p = np.empty((n, 3), dtype=float)
        result_q = np.empty((n, 3), dtype=float)
        pending = np.arange(n)
        for attempt in range(88):
            if not len(pending):
                return result_p, result_q
            if attempt < 64:
                candidates = lower[pending] + self.spacing * rng.random((len(pending), 3))
            else:
                radius = self.spacing * 0.25 * 0.5 ** (attempt - 64)
                candidates = self.points[chosen[pending]] + rng.uniform(-radius, radius, (len(pending), 3))
            q = self.model.ik(candidates)
            accepted = np.isfinite(q).all(axis=-1) & (self.cell_ids(candidates) == ids[pending])
            keep = pending[accepted]
            result_p[keep], result_q[keep] = candidates[accepted], q[accepted]
            pending = pending[~accepted]
        if len(pending):
            raise WorkspaceSamplingError(f"continuous sampling exhausted for {len(pending)} selected boundary cells")
        return result_p, result_q

    def save(self, path: str | Path) -> None:
        """Save a portable .npz; load_workspace(path, model=None) restores it."""
        path = Path(path)
        if path.suffix != ".npz":
            raise ValueError("workspace path must end in .npz")
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, points=self.points, joints=self.joints,
                            origin=self.origin, spacing=self.spacing,
                            shape=np.asarray(self.shape), mask=self.mask,
                            grid_ids=self.grid_ids,
                            metadata=json.dumps(self.metadata, sort_keys=True))

    @classmethod
    def load(cls, path: str | Path, model: DeltaModel | None = None) -> "Workspace":
        return load_workspace(path, model=model)


def _bounds(model: DeltaModel) -> tuple[np.ndarray, np.ndarray]:
    """Conservative intersection of the three moving-sphere bounding boxes.

    Trig extrema over each joint interval are exact. This bound is independent
    of candidate trajectories and does not assume the workspace is convex.
    """
    par = model.params
    centers_min, centers_max = [], []
    for leg in range(3):
        low, high = model.q_min[leg], model.q_max[leg]
        knots = np.arange(np.floor(low / (np.pi / 2)), np.ceil(high / (np.pi / 2)) + 1) * (np.pi / 2)
        angles = np.r_[low, high, knots[(knots >= low) & (knots <= high)]]
        centers = ((par.base_radius - par.platform_radius + par.upper_arm * np.cos(angles))[:, None]
                   * model.radial[leg])
        centers[:, 2] = -par.upper_arm * np.sin(angles)
        centers_min.append(centers.min(axis=0) - par.lower_arm)
        centers_max.append(centers.max(axis=0) + par.lower_arm)
    return np.max(centers_min, axis=0), np.min(centers_max, axis=0)


def _components(mask: np.ndarray) -> tuple[np.ndarray, list[int]]:
    labels = np.full(mask.size, -1, dtype=np.int32)
    marked = mask.ravel()
    sx, sy, sz = mask.shape
    sizes = []
    for start in np.flatnonzero(marked):
        if labels[start] >= 0:
            continue
        component = len(sizes)
        queue = deque([int(start)])
        labels[start] = component
        count = 0
        while queue:
            item = queue.popleft()
            count += 1
            x, remainder = divmod(item, sy * sz)
            y, z = divmod(remainder, sz)
            neighbors = []
            if x > 0: neighbors.append(item - sy * sz)
            if x + 1 < sx: neighbors.append(item + sy * sz)
            if y > 0: neighbors.append(item - sz)
            if y + 1 < sy: neighbors.append(item + sz)
            if z > 0: neighbors.append(item - 1)
            if z + 1 < sz: neighbors.append(item + 1)
            for neighbor in neighbors:
                if marked[neighbor] and labels[neighbor] < 0:
                    labels[neighbor] = component
                    queue.append(neighbor)
        sizes.append(count)
    return labels.reshape(mask.shape), sizes


def build_workspace(model: DeltaModel, spacing: float = 0.02) -> Workspace:
    """Scan Cartesian cells, supplement missed boundary cells with independent FK.

    Each cell tests its center, then a 3x3x3 lattice at fractional coordinates
    0.001, 0.5, 0.999 until a legal representative is found. Independent uniform
    joint samples provide FK cross-checks and may add missed partial cells.
    Only the six-neighbor component containing home is the operating reference;
    all discovered component sizes and exclusions are retained in metadata.
    """
    if not np.isfinite(spacing) or spacing <= 0:
        raise ValueError("spacing must be positive and finite")
    lower, upper = _bounds(model)
    origin = np.floor(lower / spacing) * spacing
    shape = tuple(np.ceil((upper - origin) / spacing).astype(int).tolist())
    if min(shape) <= 0 or np.prod(shape) > 10_000_000:
        raise ValueError("invalid or excessively large reference grid")
    coordinates = np.indices(shape).reshape(3, -1).T
    corners = origin + spacing * coordinates
    representatives = np.full((len(corners), 3), np.nan)
    joints = np.full_like(representatives, np.nan)
    valid = np.zeros(len(corners), dtype=bool)
    offsets = [np.full(3, .5)]
    offsets.extend(np.array([x, y, z]) for x in (.001, .5, .999)
                   for y in (.001, .5, .999) for z in (.001, .5, .999)
                   if (x, y, z) != (.5, .5, .5))
    scans = 0
    for offset in offsets:
        pending = np.flatnonzero(~valid)
        for start in range(0, len(pending), 8192):
            index = pending[start:start + 8192]
            p = corners[index] + spacing * offset
            q = model.ik(p)
            good = np.isfinite(q).all(axis=-1)
            selected = index[good]
            representatives[selected], joints[selected] = p[good], q[good]
            valid[selected] = True
            scans += len(index)
    cartesian_count = int(valid.sum())
    # Fixed, trajectory-independent cross-check stream. Never use experiment
    # traces or their successes/failures to change this reference denominator.
    cross_seed, cross_count = 1729, 8192
    rng = np.random.default_rng(cross_seed)
    cross_q = rng.uniform(model.q_min, model.q_max, (cross_count, 3))
    cross_p = model.fk(cross_q)
    finite = np.isfinite(cross_p).all(axis=-1)
    cross_p, cross_q = cross_p[finite], cross_q[finite]
    cell = np.floor((cross_p - origin) / spacing).astype(int)
    inside = ((cell >= 0) & (cell < shape)).all(axis=-1)
    if not inside.all():
        raise RuntimeError("independent FK cross-check escaped conservative bounding region")
    flat = np.ravel_multi_index(cell.T, shape)
    missing_ids, first = np.unique(flat[~valid[flat]], return_index=True)
    missing_p, missing_q = cross_p[~valid[flat]], cross_q[~valid[flat]]
    representatives[missing_ids], joints[missing_ids] = missing_p[first], missing_q[first]
    valid[missing_ids] = True
    all_mask = valid.reshape(shape)
    labels, sizes = _components(all_mask)
    home_cell = np.floor((model.home - origin) / spacing).astype(int)
    home_label = int(labels[tuple(home_cell)])
    if home_label < 0:
        raise RuntimeError("reference sampling missed home; refine the reference grid")
    selected_mask = labels == home_label
    grid_ids = np.flatnonzero(selected_mask)
    selected_points, selected_joints = representatives[grid_ids], joints[grid_ids]
    # Separate numerical FK residual, independent of the Cartesian input.
    cross_error = np.linalg.norm(model.fk(selected_joints) - selected_points, axis=-1)
    metadata = {
        "schema_version": 1,
        "parameter_source": "Tools/dxl_arm/generate_fixed_trajectory.py:GEOMETRY",
        "model": model.config(), "parameter_hash": model.parameter_hash,
        "coordinate_frame": "Delta base; +z up; meters and radians",
        "branch": "elbow-out analytic IK; lower three-sphere FK root",
        "reference_definition": "independent Cartesian voxel probes plus independent FK boundary cross-check; home connected component",
        "collision_scope": "ideal fixed-orientation point/link geometry; no full link-volume collision certification",
        "spacing_m": float(spacing), "origin_m": origin.tolist(), "shape": list(shape),
        "conservative_bounds_m": [lower.tolist(), upper.tolist()],
        "cell_probes": "center then 3x3x3 interior lattice at .001/.5/.999; early exit",
        "evaluated_cartesian_points": scans, "cartesian_valid_cells": cartesian_count,
        "fk_crosscheck_seed": cross_seed, "fk_crosscheck_joint_samples": cross_count,
        "fk_crosscheck_valid_samples": len(cross_p), "fk_crosscheck_out_of_bounds": 0,
        "fk_crosscheck_added_partial_cells": len(missing_ids),
        "component_neighbor_rule": "six face neighbors; approximate, not a path certificate",
        "component_sizes_cells": sizes, "selected_component": home_label,
        "all_components_cells": int(valid.sum()), "reference_cells": len(grid_ids),
        "excluded_other_components_cells": int(valid.sum() - len(grid_ids)),
        "max_fk_ik_residual_m": float(cross_error.max()),
        "volume_note": "cell count times spacing cubed is an occupied-cell proxy, not exact feasible volume",
        "partial_cell_note": "finite probes can miss arbitrarily thin feasible subsets; perform a resolution sensitivity check",
        "sampling_note": "uniform cells; uniform rejection within cell, with continuous local jitter fallback for thin cells",
    }
    return Workspace(selected_points, selected_joints, origin, float(spacing), shape,
                     selected_mask, grid_ids, metadata, model)


def load_workspace(path: str | Path, model: DeltaModel | None = None) -> Workspace:
    """Load .npz without pickle; reject an explicitly supplied mismatched model."""
    with np.load(path, allow_pickle=False) as data:
        metadata = json.loads(str(data["metadata"]))
        if metadata.get("schema_version") != 1:
            raise ValueError("unsupported workspace schema version")
        config = dict(metadata["model"])
        params = DeltaParams(**config.pop("params"))
        stored_model = DeltaModel(params, **config)
        if model is None:
            model = stored_model
        elif model.parameter_hash != stored_model.parameter_hash:
            raise ValueError("workspace and supplied model have different parameters/constraints")
        result = Workspace(data["points"].copy(), data["joints"].copy(), data["origin"].copy(),
                           float(data["spacing"]), tuple(data["shape"].astype(int).tolist()),
                           data["mask"].copy(), data["grid_ids"].copy(), metadata, model)
    if not np.array_equal(np.flatnonzero(result.mask), result.grid_ids):
        raise ValueError("workspace mask and grid IDs are inconsistent")
    return result


def save_workspace(workspace: Workspace, path: str | Path) -> None:
    workspace.save(path)
