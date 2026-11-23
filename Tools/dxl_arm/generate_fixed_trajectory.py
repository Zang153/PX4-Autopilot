#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from delta_motion.balanced import BalancedMotionConfig, BalancedRandomMotion
from delta_motion.kinematics import DeltaModel, DeltaParams
from delta_motion.trajectory import Segment
from delta_motion.workspace import build_workspace

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_DIR = REPOSITORY_ROOT / "src" / "drivers" / "dxl_arm"
SEED = 1
SPEED_BAND = 2
NORMALIZED_SPEED = 0.30
DT_S = 0.02
RANDOM_DURATION_S = 58.0
BRAKE_DURATION_S = 2.0
DURATION_S = RANDOM_DURATION_S + BRAKE_DURATION_S
SAMPLE_COUNT = round(DURATION_S / DT_S) + 1
GEOMETRY = DeltaParams(
    base_radius=0.0745,
    platform_radius=0.025,
    upper_arm=0.1,
    lower_arm=0.2,
    rod_spacing=0.036,
    home_angle_deg=0.0,
    joint_up_limit_deg=30.0,
    joint_down_limit_deg=90.0,
)


def _source_hash() -> str:
    digest = hashlib.sha256()
    paths = [Path(__file__).resolve()]
    paths.extend(sorted((Path(__file__).parent / "delta_motion").glob("*.py")))
    for path in paths:
        digest.update(path.name.encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def build_fixed_trajectory():
    model = DeltaModel(GEOMETRY)
    workspace = build_workspace(model, spacing=0.01)
    config = BalancedMotionConfig(duration=RANDOM_DURATION_S, dt=DT_S)
    generator = BalancedRandomMotion(model, workspace, config)
    activity = {
        "speed_band": SPEED_BAND,
        "target_normalized_speed": NORMALIZED_SPEED,
    }
    episode = generator.generate(
        np.zeros(3),
        np.random.default_rng(SEED),
        method="workspace",
        activity=activity,
    )
    if not episode.success:
        raise RuntimeError(f"random trajectory generation failed: {episode.diagnostics}")

    brake = Segment.connect(
        episode.q[-1],
        episode.qd[-1],
        episode.qdd[-1],
        episode.q[-1] + 0.5 * BRAKE_DURATION_S * episode.qd[-1],
        np.zeros(3),
        np.zeros(3),
        BRAKE_DURATION_S,
    )
    reason = brake.check(model, config.motion_config(model))
    if reason is not None:
        raise RuntimeError(f"final brake violates {reason}")

    brake_times = np.arange(1, round(BRAKE_DURATION_S / DT_S) + 1) * DT_S
    q = np.concatenate((episode.q, brake.evaluate(brake_times)))
    qd = np.concatenate((episode.qd, brake.evaluate(brake_times, order=1)))
    qdd = np.concatenate((episode.qdd, brake.evaluate(brake_times, order=2)))
    jerk = np.concatenate((episode.jerk, brake.evaluate(brake_times, order=3)))
    times = np.arange(SAMPLE_COUNT) * DT_S

    if q.shape != (SAMPLE_COUNT, 3) or not np.isfinite([q, qd, qdd, jerk]).all():
        raise RuntimeError("invalid trajectory shape or nonfinite samples")
    if not np.allclose(times[-1], DURATION_S, rtol=0.0, atol=1e-12):
        raise RuntimeError("trajectory does not cover exactly 60 seconds")
    if not model.valid_q(q).all() or not model.valid_q(q.astype(np.float32).astype(float)).all():
        raise RuntimeError("trajectory leaves the validated Delta workspace")
    if not np.array_equal(q[0], np.zeros(3)):
        raise RuntimeError("trajectory must start at home")
    if not np.allclose(qd[0], 0.0, rtol=0.0, atol=1e-12) or not np.allclose(
        qdd[0], 0.0, rtol=0.0, atol=1e-12
    ):
        raise RuntimeError("trajectory must start from rest")
    if not np.allclose(qd[-1], 0.0, rtol=0.0, atol=1e-12) or not np.allclose(
        qdd[-1], 0.0, rtol=0.0, atol=1e-12
    ):
        raise RuntimeError("trajectory must stop with zero velocity and acceleration")

    for order, name in ((0, "position"), (1, "velocity"), (2, "acceleration")):
        expected = (q, qd, qdd)[order]
        actual = brake.evaluate(np.array([0.0]), order=order)[0]
        reference = (episode.q, episode.qd, episode.qdd)[order][-1]
        if not np.allclose(actual, reference, rtol=0.0, atol=1e-10):
            raise RuntimeError(f"brake {name} is discontinuous at 58 seconds")

    positions = model.fk(q)
    realized_path_length = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
    quantized = q.astype("<f4")
    manifest = {
        "schema": "dxl_fixed_random_trajectory_v1",
        "algorithm": "balanced_v2_with_terminal_quintic_brake",
        "seed": SEED,
        "activity": {
            "speed_band": SPEED_BAND,
            "target_normalized_speed": NORMALIZED_SPEED,
        },
        "geometry": model.config(),
        "geometry_sha256": model.parameter_hash,
        "workspace": {
            "spacing_m": 0.01,
            "cells": len(workspace.points),
        },
        "control": {
            "frequency_hz": 1.0 / DT_S,
            "period_us": round(DT_S * 1_000_000),
            "random_duration_s": RANDOM_DURATION_S,
            "brake_duration_s": BRAKE_DURATION_S,
            "duration_s": DURATION_S,
            "sample_count": SAMPLE_COUNT,
            "joint_order": ["front_id1", "left_rear_id2", "right_rear_id3"],
            "angle_convention": "standard_radians_not_sdf",
            "home_tolerance_rad": 0.15,
        },
        "random_motion": {
            "realized_path_length_m": episode.diagnostics["realized_path_length_m"],
            "realized_mean_speed_m_s": episode.diagnostics["realized_mean_speed_m_s"],
            "time_dilation": episode.diagnostics["time_dilation"],
            "segments": episode.diagnostics["segment_count"],
        },
        "combined_path_length_m": realized_path_length,
        "q_min_rad": q.min(axis=0).tolist(),
        "q_max_rad": q.max(axis=0).tolist(),
        "max_abs_qd_rad_s": float(np.abs(qd).max()),
        "max_abs_qdd_rad_s2": float(np.abs(qdd).max()),
        "max_abs_jerk_rad_s3": float(np.abs(jerk).max()),
        "initial_q": q[0].tolist(),
        "final_q": q[-1].tolist(),
        "final_qd": qd[-1].tolist(),
        "final_qdd": qdd[-1].tolist(),
        "position_bytes": len(quantized.tobytes()),
        "position_sha256": hashlib.sha256(quantized.tobytes()).hexdigest(),
        "source_sha256": _source_hash(),
    }
    return model, config, episode, brake, times, q, qd, qdd, jerk, manifest


def _render_header(manifest) -> str:
    control = manifest["control"]
    return f"""#pragma once

#include <stdint.h>

namespace dxl
{{
inline constexpr uint32_t RANDOM_TRAJECTORY_SAMPLE_COUNT = {control['sample_count']};
inline constexpr uint32_t RANDOM_TRAJECTORY_PERIOD_US = {control['period_us']};
inline constexpr uint32_t RANDOM_TRAJECTORY_DURATION_US = {round(control['duration_s'] * 1_000_000)};
inline constexpr float RANDOM_TRAJECTORY_HOME_TOLERANCE_RAD = {control['home_tolerance_rad']:.2f}f;
extern const float RANDOM_TRAJECTORY_Q[RANDOM_TRAJECTORY_SAMPLE_COUNT][3];
}} // namespace dxl
"""


def _render_source(q, manifest) -> str:
    lines = [
        '#include "dxl_random_trajectory.hpp"',
        "",
        "namespace dxl",
        "{",
        "const float RANDOM_TRAJECTORY_Q[RANDOM_TRAJECTORY_SAMPLE_COUNT][3] = {",
    ]
    for sample in q:
        lines.append(
            "\t{"
            + ", ".join(f"{float(value):.10f}f" for value in sample)
            + "},"
        )
    lines.extend(("};", "} // namespace dxl", ""))
    return "\n".join(lines)


def render_outputs():
    *_, q, qd, qdd, jerk, manifest = build_fixed_trajectory()
    return {
        "dxl_random_trajectory.hpp": _render_header(manifest),
        "dxl_random_trajectory.cpp": _render_source(q, manifest),
        "dxl_random_trajectory.json": json.dumps(manifest, indent=2, sort_keys=True) + "\n",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = render_outputs()
    if args.check:
        mismatches = [
            name
            for name, expected in outputs.items()
            if not (args.output_dir / name).is_file()
            or (args.output_dir / name).read_text() != expected
        ]
        if mismatches:
            print(json.dumps({"status": "outdated", "files": mismatches}, indent=2))
            return 1
        print(json.dumps({"status": "current", "files": sorted(outputs)}, indent=2))
        return 0

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for name, content in outputs.items():
        (args.output_dir / name).write_text(content)
    manifest = json.loads(outputs["dxl_random_trajectory.json"])
    print(
        json.dumps(
            {
                "status": "generated",
                "output_dir": str(args.output_dir),
                "samples": manifest["control"]["sample_count"],
                "frequency_hz": manifest["control"]["frequency_hz"],
                "duration_s": manifest["control"]["duration_s"],
                "position_sha256": manifest["position_sha256"],
                "max_abs_qd_rad_s": manifest["max_abs_qd_rad_s"],
                "max_abs_qdd_rad_s2": manifest["max_abs_qdd_rad_s2"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
