#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import time

import numpy as np

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_TRAJECTORY = REPOSITORY_ROOT / "src" / "drivers" / "dxl_arm" / "dxl_random_trajectory.cpp"
DEFAULT_MODEL = "mujoco_uav_delta"
FREQUENCY_HZ = 50.0
DURATION_S = 60.0


def load_trajectory(path: Path = DEFAULT_TRAJECTORY) -> np.ndarray:
    samples = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if line.startswith("{") and line.endswith("},"):
            samples.append([float(value.rstrip("f")) for value in line[1:-2].split(",")])
    trajectory = np.asarray(samples, dtype=np.float64)
    if trajectory.shape != (3001, 3) or not np.isfinite(trajectory).all():
        raise ValueError(f"expected 3001 finite joint samples, got {trajectory.shape}")
    return trajectory


def standard_to_sdf(samples: np.ndarray) -> np.ndarray:
    return -samples[..., [2, 1, 0]]


def sample_limit(duration_s: float, sample_count: int) -> int:
    if not 0.0 < duration_s <= DURATION_S:
        raise ValueError(f"duration must be in (0, {DURATION_S}] seconds")
    return min(sample_count, round(duration_s * FREQUENCY_HZ) + 1)


def play(model: str, duration_s: float, connection_timeout_s: float) -> None:
    from gz.msgs10.double_pb2 import Double
    from gz.transport13 import Node

    trajectory = standard_to_sdf(load_trajectory())[: sample_limit(duration_s, 3001)]
    node = Node()
    publishers = [
        node.advertise(f"/{model}/delta_elbow_{index}_hinge/cmd_pos", Double)
        for index in (1, 2, 3)
    ]
    deadline = time.monotonic() + connection_timeout_s
    while time.monotonic() < deadline:
        if all(publisher.has_connections for publisher in publishers):
            break
        time.sleep(0.05)
    else:
        raise RuntimeError(
            f"no Gazebo joint-position subscribers for model {model}; "
            "start the PX4 mujoco_uav_delta world first"
        )

    start = time.monotonic()
    for index, target in enumerate(trajectory):
        delay = start + index / FREQUENCY_HZ - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        for joint, publisher in enumerate(publishers):
            message = Double()
            message.data = float(target[joint])
            publisher.publish(message)
        if index % 250 == 0:
            print(f"t={index / FREQUENCY_HZ:5.1f} s, sample={index}, sdf_q={target.tolist()}", flush=True)
    print(f"complete: {len(trajectory)} samples, holding final target", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--connection-timeout", type=float, default=10.0)
    args = parser.parse_args()
    play(args.model, args.duration, args.connection_timeout)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
