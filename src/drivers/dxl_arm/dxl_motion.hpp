#pragma once

#include <stdint.h>

namespace dxl
{

enum class ArmState : uint8_t {
	HOLD = 0,
	CENTERING = 1,
	CENTER_HOLD = 2,
	RANDOM = 3,
	FAULT = 4,
	RAISE = 5,
};

struct MotionConfig {
	float minimum[3];
	float maximum[3];
	float maximum_velocity;
	float maximum_acceleration;
	float random_amplitude;
	float center_tolerance;
	uint32_t random_seed;
};

class MotionController
{
public:
	void configure(const MotionConfig &config);
	void reset(const float actual[3]);

	bool requestCenter(const float actual[3]);
	bool requestRaise(const float actual[3]);
	bool requestRandom(const float actual[3]);
	void hold(const float actual[3]);
	void fault(const float actual[3]);

	void update(const float actual[3], float dt);

	ArmState state() const { return _state; }
	const float *target() const { return _position; }
	bool centered(const float actual[3]) const;
	bool centerQualified() const { return _center_qualified; }
	bool isMoving() const
	{
		return _state == ArmState::CENTERING || _state == ArmState::RAISE || _state == ArmState::RANDOM;
	}

private:
	void resetPlanner(const float actual[3], ArmState state);
	bool actualIsFinite(const float actual[3]) const;
	bool plannerSettled() const;
	void chooseRandomGoal();
	uint32_t nextRandom();
	float randomUnit();

	MotionConfig _config{};
	ArmState _state{ArmState::HOLD};
	bool _center_qualified{false};
	float _position[3] {};
	float _velocity[3] {};
	float _goal[3] {};
	uint32_t _rng{1};
};

} // namespace dxl
