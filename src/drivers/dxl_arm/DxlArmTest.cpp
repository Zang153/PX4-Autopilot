#include <gtest/gtest.h>

#include "dxl_protocol.hpp"
#include "dxl_rc.hpp"
#include "dxl_trajectory.hpp"

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

TEST(DxlRcTest, ChannelMatchesMeasuredThreePositionDetents)
{
	EXPECT_EQ(dxl::rcModeFromChannel(2000), dxl::RcMode::UP);
	EXPECT_EQ(dxl::rcModeFromChannel(1900), dxl::RcMode::UP);
	EXPECT_EQ(dxl::rcModeFromChannel(2100), dxl::RcMode::UP);
	EXPECT_EQ(dxl::rcModeFromChannel(1503), dxl::RcMode::CENTER);
	EXPECT_EQ(dxl::rcModeFromChannel(1403), dxl::RcMode::CENTER);
	EXPECT_EQ(dxl::rcModeFromChannel(1603), dxl::RcMode::CENTER);
	EXPECT_EQ(dxl::rcModeFromChannel(999), dxl::RcMode::DOWN);
	EXPECT_EQ(dxl::rcModeFromChannel(899), dxl::RcMode::DOWN);
	EXPECT_EQ(dxl::rcModeFromChannel(1099), dxl::RcMode::DOWN);
	EXPECT_EQ(dxl::rcModeFromChannel(1700), dxl::RcMode::UNKNOWN);
	EXPECT_EQ(dxl::rcModeFromChannel(2101), dxl::RcMode::UNKNOWN);
	EXPECT_EQ(dxl::rcModeFromChannel(898), dxl::RcMode::UNKNOWN);
}

TEST(DxlRcTest, ModesMapToDirectPositionCommands)
{
	constexpr float pi = 3.14159265358979323846f;
	float angle = 0.25f;
	ASSERT_TRUE(dxl::angleForMode(dxl::RcMode::UP, -pi / 6.0f, pi / 2.0f, angle));
	EXPECT_NEAR(angle, -pi / 6.0f, 1.0e-6f);
	EXPECT_EQ(dxl::radiansToCounts(angle, 2048, 1), 1707);
	ASSERT_TRUE(dxl::angleForMode(dxl::RcMode::CENTER, -pi / 6.0f, pi / 2.0f, angle));
	EXPECT_EQ(angle, 0.0f);
	EXPECT_EQ(dxl::radiansToCounts(angle, 2048, 1), 2048);
	ASSERT_TRUE(dxl::angleForMode(dxl::RcMode::DOWN, -pi / 6.0f, pi / 2.0f, angle));
	EXPECT_NEAR(angle, pi / 2.0f, 1.0e-6f);
	EXPECT_EQ(dxl::radiansToCounts(angle, 2048, 1), 3072);
	angle = 0.25f;
	EXPECT_FALSE(dxl::angleForMode(dxl::RcMode::UNKNOWN, -pi / 6.0f, pi / 2.0f, angle));
	EXPECT_EQ(angle, 0.25f);
}

TEST(DxlTrajectoryTest, StartRequiresValidHomeFeedback)
{
	float q_actual[3] {};
	EXPECT_FALSE(dxl::randomTrajectoryStartReady(false, q_actual));
	EXPECT_TRUE(dxl::randomTrajectoryStartReady(true, q_actual));
	EXPECT_FALSE(dxl::randomTrajectoryStartReady(true, q_actual, -0.1f));

	q_actual[1] = 0.151f;
	EXPECT_FALSE(dxl::randomTrajectoryStartReady(true, q_actual));
	q_actual[1] = 0.149f;
	EXPECT_TRUE(dxl::randomTrajectoryStartReady(true, q_actual));
	q_actual[2] = NAN;
	EXPECT_FALSE(dxl::randomTrajectoryStartReady(true, q_actual));
}

TEST(DxlTrajectoryTest, PlaysAt50HzAndHoldsFinalSample)
{
	dxl::RandomTrajectory trajectory;
	float q_target[3] {-1.0f, -1.0f, -1.0f};
	constexpr uint64_t start = 100000;
	EXPECT_FALSE(trajectory.update(start, q_target));
	ASSERT_TRUE(trajectory.start(start));
	EXPECT_TRUE(trajectory.update(start, q_target));
	EXPECT_EQ(trajectory.sampleIndex(), 0u);

	for (uint8_t i = 0; i < 3; ++i) {
		EXPECT_FLOAT_EQ(q_target[i], dxl::RANDOM_TRAJECTORY_Q[0][i]);
	}

	EXPECT_TRUE(trajectory.update(start + dxl::RANDOM_TRAJECTORY_PERIOD_US - 1, q_target));
	EXPECT_EQ(trajectory.sampleIndex(), 0u);
	EXPECT_TRUE(trajectory.update(start + dxl::RANDOM_TRAJECTORY_PERIOD_US, q_target));
	EXPECT_EQ(trajectory.sampleIndex(), 1u);

	for (uint8_t i = 0; i < 3; ++i) {
		EXPECT_FLOAT_EQ(q_target[i], dxl::RANDOM_TRAJECTORY_Q[1][i]);
	}

	EXPECT_FALSE(trajectory.start(start));
	EXPECT_TRUE(trajectory.update(start + dxl::RANDOM_TRAJECTORY_DURATION_US
				      - dxl::RANDOM_TRAJECTORY_PERIOD_US, q_target));
	EXPECT_EQ(trajectory.sampleIndex(), dxl::RANDOM_TRAJECTORY_SAMPLE_COUNT - 2);
	EXPECT_FALSE(trajectory.update(start + dxl::RANDOM_TRAJECTORY_DURATION_US, q_target));
	EXPECT_FALSE(trajectory.active());
	EXPECT_EQ(trajectory.sampleIndex(), dxl::RANDOM_TRAJECTORY_SAMPLE_COUNT - 1);

	for (uint8_t i = 0; i < 3; ++i) {
		EXPECT_FLOAT_EQ(q_target[i], dxl::RANDOM_TRAJECTORY_Q[dxl::RANDOM_TRAJECTORY_SAMPLE_COUNT - 1][i]);
	}

	EXPECT_FALSE(trajectory.update(start + dxl::RANDOM_TRAJECTORY_DURATION_US + 1000, q_target));
	ASSERT_TRUE(trajectory.start(start + dxl::RANDOM_TRAJECTORY_DURATION_US + 1000));
	EXPECT_EQ(trajectory.sampleIndex(), 0u);
}

TEST(DxlTrajectoryTest, StopAndClockRollbackAbortPlayback)
{
	dxl::RandomTrajectory trajectory;
	float q_target[3] {};
	ASSERT_TRUE(trajectory.start(100000));
	EXPECT_TRUE(trajectory.update(120000, q_target));
	trajectory.stop();
	EXPECT_FALSE(trajectory.active());
	EXPECT_FALSE(trajectory.update(140000, q_target));

	ASSERT_TRUE(trajectory.start(200000));
	EXPECT_FALSE(trajectory.update(199999, q_target));
	EXPECT_FALSE(trajectory.active());
	ASSERT_TRUE(trajectory.start(300000));
	EXPECT_TRUE(trajectory.update(300000, q_target));
}

TEST(DxlTrajectoryTest, RcAbortRequiresExplicitResetBeforeRestart)
{
	dxl::RandomTrajectory trajectory;
	ASSERT_TRUE(trajectory.start(100000));
	trajectory.abortPlayback();
	EXPECT_FALSE(trajectory.active());
	EXPECT_TRUE(trajectory.restartInhibited());
	EXPECT_FALSE(trajectory.start(200000));

	trajectory.clearRestartInhibit();
	EXPECT_FALSE(trajectory.restartInhibited());
	ASSERT_TRUE(trajectory.start(300000));
	EXPECT_TRUE(trajectory.active());
}
