#pragma once

#include <stddef.h>
#include <stdint.h>

namespace dxl
{

inline constexpr uint8_t BROADCAST_ID = 0xFE;
inline constexpr uint8_t SYNC_READ = 0x82;
inline constexpr uint8_t SYNC_WRITE = 0x83;
inline constexpr uint8_t STATUS = 0x55;
inline constexpr uint16_t GOAL_POSITION_ADDRESS = 116;
inline constexpr uint16_t PRESENT_POSITION_ADDRESS = 132;
inline constexpr uint16_t TORQUE_ENABLE_ADDRESS = 64;

struct StatusPacket {
	uint8_t id{0};
	uint8_t error{0};
	uint8_t params[16] {};
	size_t param_len{0};
};

uint16_t crc16(const uint8_t *data, size_t length);

size_t buildPacket(uint8_t id, uint8_t instruction, const uint8_t *params, size_t param_len,
		   uint8_t *output, size_t output_size);

size_t buildSyncRead(const uint8_t ids[3], uint16_t address, uint8_t data_length,
		     uint8_t *output, size_t output_size);

size_t buildSyncWritePositions(const uint8_t ids[3], const int32_t positions[3],
			       uint8_t *output, size_t output_size);

size_t buildSyncWriteTorque(const uint8_t ids[3], bool enable, uint8_t *output, size_t output_size);

size_t parseStatusPackets(const uint8_t *data, size_t data_length, StatusPacket *packets,
			  size_t max_packets, size_t &offset);

uint32_t decodeUint32(const uint8_t *data, uint8_t width);

float countsToRadians(int32_t counts, int32_t zero_counts, int direction);
int32_t radiansToCounts(float radians, int32_t zero_counts, int direction);

} // namespace dxl
