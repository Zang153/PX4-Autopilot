from pathlib import Path

import numpy as np
import pytest

from generate_fixed_trajectory import (
    BRAKE_DURATION_S,
    DT_S,
    DURATION_S,
    SAMPLE_COUNT,
    build_fixed_trajectory,
    render_outputs,
)


@pytest.fixture(scope="module")
def generated():
    return build_fixed_trajectory()


def test_fixed_trajectory_contract(generated):
    model, _, _, _, times, q, qd, qdd, jerk, manifest = generated
    assert q.shape == (SAMPLE_COUNT, 3)
    assert len(times) == SAMPLE_COUNT
    assert times[1] - times[0] == DT_S
    assert times[-1] == DURATION_S
    assert np.isfinite([q, qd, qdd, jerk]).all()
    assert model.valid_q(q).all()
    assert model.valid_q(q.astype(np.float32).astype(float)).all()
    np.testing.assert_array_equal(q[0], np.zeros(3))
    np.testing.assert_allclose(qd[0], 0.0, atol=1e-12)
    np.testing.assert_allclose(qdd[0], 0.0, atol=1e-12)
    np.testing.assert_allclose(qd[-1], 0.0, atol=1e-12)
    np.testing.assert_allclose(qdd[-1], 0.0, atol=1e-12)
    assert manifest["position_bytes"] == SAMPLE_COUNT * 3 * 4


def test_terminal_brake_is_c2_and_within_limits(generated):
    model, config, episode, brake, _, q, qd, qdd, _, _ = generated
    assert brake.duration == BRAKE_DURATION_S
    assert brake.check(model, config.motion_config(model)) is None
    for order, arrays in ((0, q), (1, qd), (2, qdd)):
        expected = (episode.q, episode.qd, episode.qdd)[order][-1]
        np.testing.assert_allclose(brake.evaluate(np.array([0.0]), order=order)[0], expected)
    np.testing.assert_allclose(brake.evaluate(np.array([brake.duration]), order=1)[0], 0.0, atol=1e-12)
    np.testing.assert_allclose(brake.evaluate(np.array([brake.duration]), order=2)[0], 0.0, atol=1e-12)


def test_exported_files_match_fixed_seed(generated):
    manifest = generated[-1]
    outputs = render_outputs()
    output_dir = Path(__file__).resolve().parents[2] / "src" / "drivers" / "dxl_arm"
    for name, expected in outputs.items():
        assert (output_dir / name).is_file()
        assert (output_dir / name).read_text() == expected
    assert manifest["position_sha256"] == outputs["dxl_random_trajectory.json"].split('"position_sha256": "')[1].split('"')[0]
