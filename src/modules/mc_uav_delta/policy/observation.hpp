#pragma once

#include <px4_platform_common/defines.h>

#include <math.h>
#include <stdint.h>

namespace mc_uav_delta
{

inline constexpr uint32_t UAV_OBSERVATION_DIM = 22;
inline constexpr uint32_t DELTA_OBSERVATION_DIM = 3;
inline constexpr uint32_t POLICY_OBSERVATION_DIM = UAV_OBSERVATION_DIM + DELTA_OBSERVATION_DIM;
inline constexpr uint32_t POLICY_ACTION_DIM = 4;

struct UavState {
	float position[3];
	float linear_velocity[3];
	float attitude[4];
	float angular_velocity[3];
};

struct UavTarget {
	float position[3];
	float linear_velocity[3];
	float yaw;
};

struct PolicyObservation {
	float position[3];
	float orientation[4];
	float linear_velocity[3];
	float angular_velocity[3];
	float previous_action[POLICY_ACTION_DIM];
	float q_actual[DELTA_OBSERVATION_DIM];
};

struct PolicyAction {
	float action[POLICY_ACTION_DIM];
};

inline float clip(float value, float maximum, float minimum)
{
	if (value > maximum) {
		return maximum;
	}

	if (value < minimum) {
		return minimum;
	}

	return value;
}

inline void quaternionMultiplication(const float q1[4], const float q2[4], float result[4])
{
	result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
}

inline void quaternionConjugate(const float q[4], float result[4])
{
	result[0] = q[0];
	result[1] = -q[1];
	result[2] = -q[2];
	result[3] = -q[3];
}

inline void quaternionToRotationMatrix(const float q[4], float rotation[9])
{
	const float qw = q[0];
	const float qx = q[1];
	const float qy = q[2];
	const float qz = q[3];

	rotation[0] = 1 - 2 * qy * qy - 2 * qz * qz;
	rotation[1] = 2 * qx * qy - 2 * qw * qz;
	rotation[2] = 2 * qx * qz + 2 * qw * qy;
	rotation[3] = 2 * qx * qy + 2 * qw * qz;
	rotation[4] = 1 - 2 * qx * qx - 2 * qz * qz;
	rotation[5] = 2 * qy * qz - 2 * qw * qx;
	rotation[6] = 2 * qx * qz - 2 * qw * qy;
	rotation[7] = 2 * qy * qz + 2 * qw * qx;
	rotation[8] = 1 - 2 * qx * qx - 2 * qy * qy;
}

inline void rotateVector(const float rotation[9], const float vector[3], float result[3])
{
	result[0] = rotation[0] * vector[0] + rotation[1] * vector[1] + rotation[2] * vector[2];
	result[1] = rotation[3] * vector[0] + rotation[4] * vector[1] + rotation[5] * vector[2];
	result[2] = rotation[6] * vector[0] + rotation[7] * vector[1] + rotation[8] * vector[2];
}

inline PolicyObservation buildPolicyObservation(const UavState &state, const UavTarget &target,
		const float previous_action[POLICY_ACTION_DIM], const float q_actual[DELTA_OBSERVATION_DIM],
		float max_position_error = 0.5f, float max_velocity_error = 1.0f)
{
	PolicyObservation observation{};
	float target_rotation_inverse[9];

	{
		float q_target[4] = {cosf(0.5f * target.yaw), 0, 0, sinf(0.5f * target.yaw)};
		float qt[4] = {q_target[0], q_target[1], -q_target[2], -q_target[3]};
		float qtc[4];
		quaternionConjugate(qt, qtc);
		quaternionToRotationMatrix(qtc, target_rotation_inverse);

		const float qr[4] = {state.attitude[0], state.attitude[1], -state.attitude[2], -state.attitude[3]};
		quaternionMultiplication(qtc, qr, observation.orientation);
	}

	{
		const float error[3] = {
			state.position[0] - target.position[0],
			-(state.position[1] - target.position[1]),
			-(state.position[2] - target.position[2])
		};
		float target_frame_error[3];
		rotateVector(target_rotation_inverse, error, target_frame_error);

		for (uint32_t i = 0; i < 3; ++i) {
			observation.position[i] = clip(target_frame_error[i], max_position_error, -max_position_error);
		}
	}

	{
		const float error[3] = {
			state.linear_velocity[0] - target.linear_velocity[0],
			-(state.linear_velocity[1] - target.linear_velocity[1]),
			-(state.linear_velocity[2] - target.linear_velocity[2])
		};
		float target_frame_error[3];
		rotateVector(target_rotation_inverse, error, target_frame_error);

		for (uint32_t i = 0; i < 3; ++i) {
			observation.linear_velocity[i] = clip(target_frame_error[i], max_velocity_error, -max_velocity_error);
		}
	}

	observation.angular_velocity[0] = state.angular_velocity[0];
	observation.angular_velocity[1] = -state.angular_velocity[1];
	observation.angular_velocity[2] = -state.angular_velocity[2];

	for (uint32_t i = 0; i < POLICY_ACTION_DIM; ++i) {
		observation.previous_action[i] = previous_action[i];
	}

	for (uint32_t i = 0; i < DELTA_OBSERVATION_DIM; ++i) {
		observation.q_actual[i] = q_actual[i];
	}

	return observation;
}

inline void flattenObservation(const PolicyObservation &observation, const float action_history[POLICY_ACTION_DIM],
			       float output[POLICY_OBSERVATION_DIM])
{
	uint32_t index = 0;

	for (uint32_t i = 0; i < 3; ++i) {
		output[index++] = observation.position[i];
	}

	float rotation[9];
	quaternionToRotationMatrix(observation.orientation, rotation);

	for (uint32_t i = 0; i < 9; ++i) {
		output[index++] = rotation[i];
	}

	for (uint32_t i = 0; i < 3; ++i) {
		output[index++] = observation.linear_velocity[i];
	}

	for (uint32_t i = 0; i < 3; ++i) {
		output[index++] = observation.angular_velocity[i];
	}

	for (uint32_t i = 0; i < POLICY_ACTION_DIM; ++i) {
		output[index++] = action_history[i];
	}

	for (uint32_t i = 0; i < DELTA_OBSERVATION_DIM; ++i) {
		output[index++] = observation.q_actual[i];
	}
}

inline bool finiteDeltaAngles(const float q_actual[DELTA_OBSERVATION_DIM])
{
	for (uint32_t i = 0; i < DELTA_OBSERVATION_DIM; ++i) {
		if (!PX4_ISFINITE(q_actual[i])) {
			return false;
		}
	}

	return true;
}

inline bool deltaStateIsStale(bool sequence_set, bool stream_valid, uint64_t current_time,
			      uint64_t timestamp_last_delta, uint64_t max_age)
{
	return !sequence_set || !stream_valid || current_time < timestamp_last_delta
	       || current_time - timestamp_last_delta > max_age;
}

class ActionHistory
{
public:
	void reset()
	{
		_steps_since_native = 0;

		for (float &value : _mean) {
			value = 0.0f;
		}
	}

	void update(const float previous_action[POLICY_ACTION_DIM])
	{
		if (_steps_since_native == 0) {
			for (uint32_t i = 0; i < POLICY_ACTION_DIM; ++i) {
				_mean[i] = previous_action[i];
			}

		} else {
			for (uint32_t i = 0; i < POLICY_ACTION_DIM; ++i) {
				_mean[i] = (_mean[i] * _steps_since_native + previous_action[i]) / (_steps_since_native + 1);
			}
		}
	}

	void complete(bool native_step)
	{
		if (native_step) {
			_steps_since_native = 0;

		} else {
			++_steps_since_native;
		}
	}

	const float *mean() const { return _mean; }
	uint32_t stepsSinceNative() const { return _steps_since_native; }

private:
	float _mean[POLICY_ACTION_DIM] {};
	uint32_t _steps_since_native{0};
};

} // namespace mc_uav_delta
