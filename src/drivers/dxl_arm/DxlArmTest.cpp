#include <gtest/gtest.h>

#include "dxl_motion.hpp"
#include "dxl_protocol.hpp"

#include <math.h>
#include <string.h>

TEST(DxlProtocolTest, PingPacketHasValidLayoutAndCrc)
{
	uint8_t packet[16] {};
	const size_t length = dxl::buildPacket(1, 0x01, nullptr, 0, packet, sizeof(packet));
	ASSERT_EQ(length, 10u);
	EXPECT_EQ(packet[0], 0xFF);
	EXPECT_EQ(packet[1], 0xFF);
	EXPECT_EQ(packet[2], 0xFD);
	EXPECT_EQ(packet[3], 0x00);
	EXPECT_EQ(packet[4], 1);
	EXPECT_EQ(packet[5], 3);
	EXPECT_EQ(packet[6], 0);
	EXPECT_EQ(packet[7], 0x01);
	const uint16_t received_crc = packet[8] | (static_cast<uint16_t>(packet[9]) << 8);
	EXPECT_EQ(received_crc, dxl::crc16(packet, 8));
}

TEST(DxlProtocolTest, SyncReadContainsAddressWidthAndIds)
{
	const uint8_t ids[3] = {3, 1, 2};
	uint8_t packet[32] {};
	const size_t length = dxl::buildSyncRead(ids, 132, 4, packet, sizeof(packet));
	ASSERT_GT(length, 15u);
	EXPECT_EQ(packet[4], dxl::BROADCAST_ID);
	EXPECT_EQ(packet[7], dxl::SYNC_READ);
	EXPECT_EQ(packet[8], 132);
	EXPECT_EQ(packet[9], 0);
	EXPECT_EQ(packet[10], 4);
	EXPECT_EQ(packet[11], 0);
	EXPECT_EQ(packet[12], 3);
	EXPECT_EQ(packet[13], 1);
	EXPECT_EQ(packet[14], 2);
}

TEST(DxlProtocolTest, SyncWriteUsesLittleEndianPositions)
{
	const uint8_t ids[3] = {1, 2, 3};
	const int32_t positions[3] = {2048, 1707, 4095};
	uint8_t packet[48] {};
	const size_t length = dxl::buildSyncWritePositions(ids, positions, packet, sizeof(packet));
	ASSERT_EQ(length, 29u);
	EXPECT_EQ(packet[7], dxl::SYNC_WRITE);
	EXPECT_EQ(packet[8], 116);
	EXPECT_EQ(packet[10], 4);
	EXPECT_EQ(packet[12], 1);
	EXPECT_EQ(dxl::decodeUint32(packet + 13, 4), 2048u);
	EXPECT_EQ(packet[17], 2);
	EXPECT_EQ(dxl::decodeUint32(packet + 18, 4), 1707u);
	EXPECT_EQ(packet[22], 3);
	EXPECT_EQ(dxl::decodeUint32(packet + 23, 4), 4095u);
}

TEST(DxlProtocolTest, RawAnglesMatchTrainingContract)
{
	constexpr float pi = 3.14159265358979323846f;
	const float expected_1707 = (1707 - 2048) * 2.0f * pi / 4096.0f;
	EXPECT_NEAR(dxl::countsToRadians(1707, 2048, 1), expected_1707, 1.0e-6f);
	EXPECT_NEAR(dxl::countsToRadians(1707, 2048, 1), -pi / 6.0f, 6.0e-4f);
	EXPECT_NEAR(dxl::countsToRadians(2048, 2048, 1), 0.0f, 1.0e-6f);
	EXPECT_NEAR(dxl::countsToRadians(3072, 2048, 1), pi / 2.0f, 1.0e-6f);
	EXPECT_EQ(dxl::radiansToCounts(-pi / 6.0f, 2048, 1), 1707);
	EXPECT_EQ(dxl::radiansToCounts(0.0f, 2048, 1), 2048);
	EXPECT_EQ(dxl::radiansToCounts(pi / 2.0f, 2048, 1), 3072);
}

TEST(DxlProtocolTest, StatusRoundTripRemovesByteStuffing)
{
	const uint8_t params[5] = {0, 0xFF, 0xFF, 0xFD, 0x01};
	uint8_t frame[48] {};
	const size_t frame_length = dxl::buildPacket(7, dxl::STATUS, params, sizeof(params), frame, sizeof(frame));
	uint8_t input[52] = {0x12, 0x34, 0x56};
	memcpy(input + 3, frame, frame_length);
	dxl::StatusPacket packets[1] {};
	size_t offset = 0;
	ASSERT_EQ(dxl::parseStatusPackets(input, frame_length + 3, packets, 1, offset), 1u);
	EXPECT_EQ(packets[0].id, 7);
	EXPECT_EQ(packets[0].error, 0);
	ASSERT_EQ(packets[0].param_len, 4u);
	EXPECT_EQ(dxl::decodeUint32(packets[0].params, 4), 0x01FDFFFFu);
}

TEST(DxlProtocolTest, PartialAndCorruptFramesAreNotAccepted)
{
	const uint8_t params[5] = {0, 1, 2, 3, 4};
	uint8_t frame[32] {};
	const size_t length = dxl::buildPacket(2, dxl::STATUS, params, sizeof(params), frame, sizeof(frame));
	dxl::StatusPacket packets[1] {};
	size_t offset = 0;
	EXPECT_EQ(dxl::parseStatusPackets(frame, length / 2, packets, 1, offset), 0u);
	offset = 0;
	ASSERT_EQ(dxl::parseStatusPackets(frame, length, packets, 1, offset), 1u);
	frame[length - 1] ^= 0x01;
	offset = 0;
	EXPECT_EQ(dxl::parseStatusPackets(frame, length, packets, 1, offset), 0u);
}

static dxl::MotionConfig testConfig()
{
	dxl::MotionConfig config{};

	for (uint8_t i = 0; i < 3; ++i) {
		config.minimum[i] = -0.5f;
		config.maximum[i] = 1.5f;
	}

	config.maximum_velocity = 0.5f;
	config.maximum_acceleration = 2.0f;
	config.random_amplitude = 0.1f;
	config.center_tolerance = 0.02f;
	config.random_seed = 7;
	return config;
}

TEST(DxlMotionTest, CenteringIsBoundedAndRequiresActualCenter)
{
	dxl::MotionController motion;
	motion.configure(testConfig());
	const float initial[3] = {0.4f, -0.3f, 0.2f};
	motion.reset(initial);
	ASSERT_TRUE(motion.requestCenter(initial));
	float previous[3] = {initial[0], initial[1], initial[2]};
	float actual[3] = {initial[0], initial[1], initial[2]};

	for (int step = 0; step < 2000; ++step) {
		motion.update(actual, 0.01f);

		for (uint8_t i = 0; i < 3; ++i) {
			EXPECT_LE(fabsf(motion.target()[i] - previous[i]), 0.0051f);
			previous[i] = motion.target()[i];
		}
	}

	for (uint8_t i = 0; i < 3; ++i) {
		EXPECT_NEAR(motion.target()[i], 0.0f, 1.0e-4f);
	}

	EXPECT_EQ(motion.state(), dxl::ArmState::CENTERING);
	const float center[3] = {};
	motion.update(center, 0.01f);
	EXPECT_EQ(motion.state(), dxl::ArmState::CENTER_HOLD);
}

TEST(DxlMotionTest, RandomRequiresCenterAndStaysBounded)
{
	dxl::MotionController motion;
	motion.configure(testConfig());
	const float center[3] = {};
	motion.reset(center);
	EXPECT_FALSE(motion.requestRandom(center));
	ASSERT_TRUE(motion.requestCenter(center));
	motion.update(center, 0.01f);
	ASSERT_EQ(motion.state(), dxl::ArmState::CENTER_HOLD);
	ASSERT_TRUE(motion.requestRandom(center));

	for (int step = 0; step < 2000; ++step) {
		motion.update(center, 0.01f);

		for (uint8_t i = 0; i < 3; ++i) {
			EXPECT_LE(fabsf(motion.target()[i]), 0.1001f);
			EXPECT_GE(motion.target()[i], -0.5f);
			EXPECT_LE(motion.target()[i], 1.5f);
		}
	}
}

TEST(DxlMotionTest, FaultOnlyRecoversThroughCentering)
{
	dxl::MotionController motion;
	motion.configure(testConfig());
	const float center[3] = {};
	motion.reset(center);
	ASSERT_TRUE(motion.requestCenter(center));
	motion.update(center, 0.01f);
	ASSERT_TRUE(motion.requestRandom(center));
	motion.fault(center);
	EXPECT_EQ(motion.state(), dxl::ArmState::FAULT);
	EXPECT_FALSE(motion.requestRandom(center));
	EXPECT_TRUE(motion.requestCenter(center));
	EXPECT_EQ(motion.state(), dxl::ArmState::CENTERING);
	motion.hold(center);
	EXPECT_EQ(motion.state(), dxl::ArmState::HOLD);
}
