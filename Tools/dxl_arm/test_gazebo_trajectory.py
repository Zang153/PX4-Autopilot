import numpy as np

from play_gazebo_trajectory import load_trajectory, sample_limit, standard_to_sdf


def test_firmware_trajectory_is_used_directly():
    trajectory = load_trajectory()
    assert trajectory.shape == (3001, 3)
    np.testing.assert_array_equal(trajectory[0], np.zeros(3))
    assert np.isfinite(trajectory).all()


def test_sdf_conversion_reorders_and_negates_standard_angles():
    standard = np.array([[0.1, 0.2, 0.3], [-0.4, 0.5, -0.6]])
    np.testing.assert_allclose(standard_to_sdf(standard), [[-0.3, -0.2, -0.1], [0.6, -0.5, 0.4]])
    trajectory = load_trajectory()
    sdf = standard_to_sdf(trajectory)
    np.testing.assert_array_equal(sdf[0], np.zeros(3))
    np.testing.assert_allclose(sdf[-1], -trajectory[-1][[2, 1, 0]])


def test_duration_selects_samples_without_changing_the_table():
    assert sample_limit(0.5, 3001) == 26
    assert sample_limit(60.0, 3001) == 3001
