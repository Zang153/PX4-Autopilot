#include "dxl_trajectory.hpp"

namespace dxl
{

bool RandomTrajectory::start(uint64_t timestamp_us)
{
	if (_active) {
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
