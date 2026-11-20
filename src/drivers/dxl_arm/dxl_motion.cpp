#include "dxl_motion.hpp"

#include <math.h>

namespace dxl
{

void MotionController::configure(const MotionConfig &config)
{
	_config = config;
	_rng = config.random_seed == 0 ? 1 : config.random_seed;
}

void MotionController::resetPlanner(const float actual[3], ArmState state)
{
	_state = state;

	for (uint8_t i = 0; i < 3; ++i) {
		_position[i] = actual[i];
		_goal[i] = actual[i];
		_velocity[i] = 0.0f;
	}
}

void MotionController::reset(const float actual[3])
{
	resetPlanner(actual, ArmState::HOLD);
}

bool MotionController::requestCenter(const float actual[3])
{
	for (uint8_t i = 0; i < 3; ++i) {
		if (!isfinite(actual[i])) {
			return false;
		}
	}

	resetPlanner(actual, ArmState::CENTERING);

	for (uint8_t i = 0; i < 3; ++i) {
		_goal[i] = 0.0f;
	}

	return true;
}

bool MotionController::requestRandom(const float actual[3])
{
	if (_state != ArmState::CENTER_HOLD || !centered(actual)) {
		return false;
	}

	resetPlanner(actual, ArmState::RANDOM);
	return true;
}

void MotionController::hold(const float actual[3])
{
	resetPlanner(actual, ArmState::HOLD);
}

void MotionController::fault(const float actual[3])
{
	resetPlanner(actual, ArmState::FAULT);
}

bool MotionController::centered(const float actual[3]) const
{
	for (uint8_t i = 0; i < 3; ++i) {
		if (!isfinite(actual[i]) || fabsf(actual[i]) > _config.center_tolerance) {
			return false;
		}
	}

	return true;
}

bool MotionController::plannerSettled() const
{
	for (uint8_t i = 0; i < 3; ++i) {
		if (fabsf(_goal[i] - _position[i]) > 1.0e-4f || fabsf(_velocity[i]) > 1.0e-3f) {
			return false;
		}
	}

	return true;
}

uint32_t MotionController::nextRandom()
{
	_rng ^= _rng << 13;
	_rng ^= _rng >> 17;
	_rng ^= _rng << 5;
	return _rng;
}

float MotionController::randomUnit()
{
	return static_cast<float>(nextRandom()) / 4294967295.0f;
}

void MotionController::chooseRandomGoal()
{
	for (uint8_t i = 0; i < 3; ++i) {
		const float target = (2.0f * randomUnit() - 1.0f) * _config.random_amplitude;
		_goal[i] = fminf(_config.maximum[i], fmaxf(_config.minimum[i], target));
	}
}

void MotionController::update(const float actual[3], float dt)
{
	if (_state != ArmState::CENTERING && _state != ArmState::RANDOM) {
		return;
	}

	if (!isfinite(dt) || dt <= 0.0f) {
		return;
	}

	dt = fminf(dt, 0.05f);

	if (_state == ArmState::RANDOM && plannerSettled()) {
		chooseRandomGoal();
	}

	for (uint8_t i = 0; i < 3; ++i) {
		const float error = _goal[i] - _position[i];

		if (fabsf(error) < 1.0e-4f && fabsf(_velocity[i]) <= _config.maximum_acceleration * dt) {
			_position[i] = _goal[i];
			_velocity[i] = 0.0f;
			continue;
		}

		const float direction = error >= 0.0f ? 1.0f : -1.0f;
		const float braking_velocity = sqrtf(2.0f * _config.maximum_acceleration * fabsf(error));
		const float desired_velocity = direction * fminf(_config.maximum_velocity, braking_velocity);
		const float velocity_change = desired_velocity - _velocity[i];
		const float maximum_change = _config.maximum_acceleration * dt;
		_velocity[i] += fminf(maximum_change, fmaxf(-maximum_change, velocity_change));
		const float next_position = _position[i] + _velocity[i] * dt;

		if ((_goal[i] - next_position) * direction < 0.0f) {
			_position[i] = _goal[i];
			_velocity[i] = 0.0f;

		} else {
			_position[i] = next_position;
		}
	}

	if (_state == ArmState::CENTERING && plannerSettled() && centered(actual)) {
		_state = ArmState::CENTER_HOLD;
	}
}

} // namespace dxl
