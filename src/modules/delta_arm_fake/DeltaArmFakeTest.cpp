#include <gtest/gtest.h>

#include "delta_arm_fake.hpp"

TEST(DeltaArmFakeTest, ZeroStateMatchesObservationContract)
{
	constexpr hrt_abstime timestamp = 123456;
	constexpr uint32_t sequence = 42;
	const auto state = delta_arm_fake::zeroState(sequence, timestamp);

	EXPECT_EQ(state.timestamp, timestamp);
	EXPECT_EQ(state.timestamp_sample, timestamp);
	EXPECT_EQ(state.sequence, sequence);
	EXPECT_TRUE(state.valid);

	for (float angle : state.q_actual) {
		EXPECT_EQ(angle, 0.0f);
	}
}
