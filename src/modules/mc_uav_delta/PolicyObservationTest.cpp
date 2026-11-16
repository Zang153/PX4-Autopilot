#include <gtest/gtest.h>

#include <rl_tools/operations/cpu.h>

#include "policy/observation.hpp"
#include "policy/policy.hpp"

#include <math.h>

using namespace mc_uav_delta;

TEST(PolicyObservationTest, NetworkContract)
{
	EXPECT_EQ(policy::TYPE::INPUT_SHAPE::LAST, 25u);
	EXPECT_EQ(policy::TYPE::OUTPUT_SHAPE::LAST, 4u);
	EXPECT_EQ(policy::layer_2::CONFIG::ACTIVATION_FUNCTION,
		  rl_tools::nn::activation_functions::ActivationFunction::TANH);
}

TEST(PolicyObservationTest, IdentityObservationAndRawDelta)
{
	UavState state{};
	state.attitude[0] = 1.0f;
	UavTarget target{};
	const float previous_action[POLICY_ACTION_DIM] = {0.1f, -0.2f, 0.3f, -0.4f};
	const float q_actual[DELTA_OBSERVATION_DIM] = {-4.0f, 2.0f, 8.0f};

	const auto observation = buildPolicyObservation(state, target, previous_action, q_actual);
	float flattened[POLICY_OBSERVATION_DIM];
	flattenObservation(observation, previous_action, flattened);

	const float expected_rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

	for (int i = 0; i < 3; ++i) {
		EXPECT_FLOAT_EQ(flattened[i], 0.0f);
		EXPECT_FLOAT_EQ(flattened[12 + i], 0.0f);
		EXPECT_FLOAT_EQ(flattened[15 + i], 0.0f);
	}

	for (int i = 0; i < 9; ++i) {
		EXPECT_FLOAT_EQ(flattened[3 + i], expected_rotation[i]);
	}

	for (uint32_t i = 0; i < POLICY_ACTION_DIM; ++i) {
		EXPECT_FLOAT_EQ(flattened[18 + i], previous_action[i]);
	}

	for (uint32_t i = 0; i < DELTA_OBSERVATION_DIM; ++i) {
		EXPECT_FLOAT_EQ(flattened[22 + i], q_actual[i]);
	}
}

TEST(PolicyObservationTest, TargetYawRotationAndRaptorClipping)
{
	constexpr float pi = 3.14159265358979323846f;
	UavState state{};
	state.position[0] = 2.0f;
	state.linear_velocity[0] = 2.0f;
	state.attitude[0] = 1.0f;
	state.angular_velocity[0] = 0.1f;
	state.angular_velocity[1] = 0.2f;
	state.angular_velocity[2] = 0.3f;
	UavTarget target{};
	target.yaw = pi / 2.0f;
	const float previous_action[POLICY_ACTION_DIM] = {};
	const float q_actual[DELTA_OBSERVATION_DIM] = {};

	const auto observation = buildPolicyObservation(state, target, previous_action, q_actual);

	EXPECT_NEAR(observation.position[0], 0.0f, 1e-6f);
	EXPECT_NEAR(observation.position[1], 0.5f, 1e-6f);
	EXPECT_NEAR(observation.position[2], 0.0f, 1e-6f);
	EXPECT_NEAR(observation.linear_velocity[0], 0.0f, 1e-6f);
	EXPECT_NEAR(observation.linear_velocity[1], 1.0f, 1e-6f);
	EXPECT_NEAR(observation.linear_velocity[2], 0.0f, 1e-6f);
	EXPECT_NEAR(observation.orientation[0], sqrtf(0.5f), 1e-6f);
	EXPECT_NEAR(observation.orientation[1], 0.0f, 1e-6f);
	EXPECT_NEAR(observation.orientation[2], 0.0f, 1e-6f);
	EXPECT_NEAR(observation.orientation[3], sqrtf(0.5f), 1e-6f);
	EXPECT_FLOAT_EQ(observation.angular_velocity[0], 0.1f);
	EXPECT_FLOAT_EQ(observation.angular_velocity[1], -0.2f);
	EXPECT_FLOAT_EQ(observation.angular_velocity[2], -0.3f);
}

TEST(PolicyObservationTest, ActionHistoryAverageAndNativeReset)
{
	ActionHistory history;
	history.reset();
	const float first[POLICY_ACTION_DIM] = {1.0f, 0.0f, -1.0f, 0.5f};
	const float second[POLICY_ACTION_DIM] = {0.0f, 1.0f, 0.0f, -0.5f};
	const float third[POLICY_ACTION_DIM] = {1.0f, 1.0f, 1.0f, 1.0f};

	history.update(first);
	EXPECT_EQ(history.stepsSinceNative(), 0u);
	history.complete(false);
	history.update(second);
	EXPECT_EQ(history.stepsSinceNative(), 1u);
	EXPECT_FLOAT_EQ(history.mean()[0], 0.5f);
	EXPECT_FLOAT_EQ(history.mean()[1], 0.5f);
	EXPECT_FLOAT_EQ(history.mean()[2], -0.5f);
	EXPECT_FLOAT_EQ(history.mean()[3], 0.0f);

	history.complete(false);
	history.update(third);
	EXPECT_NEAR(history.mean()[0], 2.0f / 3.0f, 1e-6f);
	EXPECT_NEAR(history.mean()[1], 2.0f / 3.0f, 1e-6f);
	EXPECT_NEAR(history.mean()[2], 0.0f, 1e-6f);
	EXPECT_NEAR(history.mean()[3], 1.0f / 3.0f, 1e-6f);

	history.complete(true);
	EXPECT_EQ(history.stepsSinceNative(), 0u);
	history.update(second);
	EXPECT_FLOAT_EQ(history.mean()[0], second[0]);
	EXPECT_FLOAT_EQ(history.mean()[1], second[1]);
	EXPECT_FLOAT_EQ(history.mean()[2], second[2]);
	EXPECT_FLOAT_EQ(history.mean()[3], second[3]);

	history.reset();
	EXPECT_EQ(history.stepsSinceNative(), 0u);

	for (uint32_t i = 0; i < POLICY_ACTION_DIM; ++i) {
		EXPECT_FLOAT_EQ(history.mean()[i], 0.0f);
	}
}

TEST(PolicyObservationTest, DeltaValidityAndFreshness)
{
	const float valid[DELTA_OBSERVATION_DIM] = {-10.0f, 0.0f, 10.0f};
	EXPECT_TRUE(finiteDeltaAngles(valid));

	const float with_nan[DELTA_OBSERVATION_DIM] = {0.0f, NAN, 0.0f};
	EXPECT_FALSE(finiteDeltaAngles(with_nan));

	const float with_infinity[DELTA_OBSERVATION_DIM] = {0.0f, INFINITY, 0.0f};
	EXPECT_FALSE(finiteDeltaAngles(with_infinity));

	EXPECT_TRUE(deltaStateIsStale(false, false, 1000, 900, 100));
	EXPECT_TRUE(deltaStateIsStale(true, false, 1000, 900, 100));
	EXPECT_FALSE(deltaStateIsStale(true, true, 1000, 900, 100));
	EXPECT_FALSE(deltaStateIsStale(true, true, 1000, 950, 50));
	EXPECT_TRUE(deltaStateIsStale(true, true, 1001, 950, 50));
	EXPECT_TRUE(deltaStateIsStale(true, true, 900, 950, 50));
}
