#include "dxl_trajectory.hpp"

#include <math.h>

namespace dxl
{

bool randomTrajectoryStartReady(bool feedback_valid, const float q_actual[3], float tolerance)
{
	if (!feedback_valid || q_actual == nullptr || !isfinite(tolerance) || tolerance < 0.0f) {
		return false;
	}

	for (uint8_t i = 0; i < 3; ++i) {
		if (!isfinite(q_actual[i])
		    || fabsf(q_actual[i] - RANDOM_TRAJECTORY_Q[0][i]) > tolerance) {
			return false;
		}
	}

	return true;
}

bool RandomTrajectory::start(uint64_t timestamp_us)
{
	if (_active || _restart_inhibited) {
		return false;
	}

	_active = true;
	_start_time = timestamp_us;
	_sample_index = 0;
	return true;
}

bool RandomTrajectory::update(uint64_t timestamp_us, float q_target[3])
{
	if (!_active) {
		return false;
	}

	if (timestamp_us < _start_time) {
		stop();
		return false;
	}

	const uint64_t elapsed = timestamp_us - _start_time;
	const uint64_t sample = elapsed / RANDOM_TRAJECTORY_PERIOD_US;

	if (sample >= RANDOM_TRAJECTORY_SAMPLE_COUNT - 1) {
		_sample_index = RANDOM_TRAJECTORY_SAMPLE_COUNT - 1;

	} else {
		_sample_index = static_cast<uint32_t>(sample);
	}

	for (uint8_t i = 0; i < 3; ++i) {
		q_target[i] = RANDOM_TRAJECTORY_Q[_sample_index][i];
	}

	if (_sample_index == RANDOM_TRAJECTORY_SAMPLE_COUNT - 1) {
		stop();
		return false;
	}

	return true;
}

} // namespace dxl
