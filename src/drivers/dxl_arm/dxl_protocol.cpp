#include "dxl_protocol.hpp"

#include <math.h>
#include <string.h>

namespace dxl
{

uint16_t crc16(const uint8_t *data, size_t length)
{
	uint16_t crc = 0;

	for (size_t i = 0; i < length; ++i) {
		crc ^= static_cast<uint16_t>(data[i]) << 8;

		for (uint8_t bit = 0; bit < 8; ++bit) {
			if (crc & 0x8000) {
				crc = static_cast<uint16_t>((crc << 1) ^ 0x8005);

			} else {
				crc = static_cast<uint16_t>(crc << 1);
			}
		}
	}

	return crc;
}

size_t buildPacket(uint8_t id, uint8_t instruction, const uint8_t *params, size_t param_len,
		   uint8_t *output, size_t output_size)
{
	if (output == nullptr || output_size < 10 || (param_len > 0 && params == nullptr)) {
		return 0;
	}

	output[0] = 0xFF;
	output[1] = 0xFF;
	output[2] = 0xFD;
	output[3] = 0x00;
	output[4] = id;

	size_t index = 7;
	uint8_t ff_count = 0;

	for (size_t i = 0; i < param_len + 1; ++i) {
		const uint8_t value = i == 0 ? instruction : params[i - 1];

		if (index + 2 > output_size) {
			return 0;
		}

		output[index++] = value;

		if (value == 0xFF) {
			if (ff_count < 2) {
				++ff_count;
			}

		} else if (value == 0xFD && ff_count == 2) {
			if (index + 2 > output_size) {
				return 0;
			}

			output[index++] = 0xFD;
			ff_count = 0;

		} else {
			ff_count = 0;
		}
	}

	const uint16_t packet_length = static_cast<uint16_t>((index - 7) + 2);
	output[5] = static_cast<uint8_t>(packet_length & 0xFF);
	output[6] = static_cast<uint8_t>((packet_length >> 8) & 0xFF);
	const uint16_t crc = crc16(output, index);
	output[index++] = static_cast<uint8_t>(crc & 0xFF);
	output[index++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
	return index;
}

size_t buildSyncRead(const uint8_t ids[3], uint16_t address, uint8_t data_length,
		     uint8_t *output, size_t output_size)
{
	uint8_t params[7] = {
		static_cast<uint8_t>(address & 0xFF),
		static_cast<uint8_t>(address >> 8),
		data_length,
		0,
		ids[0], ids[1], ids[2]
	};
	return buildPacket(BROADCAST_ID, SYNC_READ, params, sizeof(params), output, output_size);
}

size_t buildSyncWritePositions(const uint8_t ids[3], const int32_t positions[3],
			       uint8_t *output, size_t output_size)
{
	uint8_t params[19] = {GOAL_POSITION_ADDRESS & 0xFF, GOAL_POSITION_ADDRESS >> 8, 4, 0};

	for (size_t i = 0; i < 3; ++i) {
		params[4 + 5 * i] = ids[i];

		for (uint8_t byte = 0; byte < 4; ++byte) {
			params[5 + 5 * i + byte] = static_cast<uint8_t>(static_cast<uint32_t>(positions[i]) >> (8 * byte));
		}
	}

	return buildPacket(BROADCAST_ID, SYNC_WRITE, params, sizeof(params), output, output_size);
}

size_t buildSyncWriteTorque(const uint8_t ids[3], bool enable, uint8_t *output, size_t output_size)
{
	uint8_t params[10] = {TORQUE_ENABLE_ADDRESS & 0xFF, TORQUE_ENABLE_ADDRESS >> 8, 1, 0};

	for (size_t i = 0; i < 3; ++i) {
		params[4 + 2 * i] = ids[i];
		params[5 + 2 * i] = enable ? 1 : 0;
	}

	return buildPacket(BROADCAST_ID, SYNC_WRITE, params, sizeof(params), output, output_size);
}

size_t parseStatusPackets(const uint8_t *data, size_t data_length, StatusPacket *packets,
			  size_t max_packets, size_t &offset)
{
	size_t count = 0;

	while (offset + 7 <= data_length && count < max_packets) {
		if (data[offset] != 0xFF || data[offset + 1] != 0xFF || data[offset + 2] != 0xFD
		    || data[offset + 3] != 0x00) {
			++offset;
			continue;
		}

		const uint16_t packet_length = static_cast<uint16_t>(data[offset + 5])
					       | (static_cast<uint16_t>(data[offset + 6]) << 8);

		if (packet_length < 4 || packet_length > 64) {
			++offset;
			continue;
		}

		const size_t frame_length = 7 + packet_length;

		if (offset + frame_length > data_length) {
			break;
		}

		const uint16_t received_crc = static_cast<uint16_t>(data[offset + frame_length - 2])
					      | (static_cast<uint16_t>(data[offset + frame_length - 1]) << 8);
		const uint16_t calculated_crc = crc16(data + offset, frame_length - 2);

		if (received_crc != calculated_crc) {
			++offset;
			continue;
		}

		if (data[offset + 7] == STATUS) {
			const uint8_t *stuffed = data + offset + 7;
			const size_t stuffed_length = packet_length - 2;
			uint8_t unstuffed[64] {};
			size_t unstuffed_length = 0;

			for (size_t i = 0; i < stuffed_length && unstuffed_length < sizeof(unstuffed); ++i) {
				if (i + 3 < stuffed_length && stuffed[i] == 0xFF && stuffed[i + 1] == 0xFF
				    && stuffed[i + 2] == 0xFD && stuffed[i + 3] == 0xFD) {
					unstuffed[unstuffed_length++] = 0xFF;
					unstuffed[unstuffed_length++] = 0xFF;
					unstuffed[unstuffed_length++] = 0xFD;
					i += 3;

				} else {
					unstuffed[unstuffed_length++] = stuffed[i];
				}
			}

			if (unstuffed_length >= 2 && unstuffed[0] == STATUS) {
				const size_t param_length = unstuffed_length - 2;

				if (param_length <= sizeof(packets[count].params)) {
					packets[count].id = data[offset + 4];
					packets[count].error = unstuffed[1];
					packets[count].param_len = param_length;

					if (param_length > 0) {
						memcpy(packets[count].params, unstuffed + 2, param_length);
					}

					++count;
				}
			}
		}

		offset += frame_length;
	}

	return count;
}

uint32_t decodeUint32(const uint8_t *data, uint8_t width)
{
	uint32_t value = 0;

	for (uint8_t i = 0; i < width && i < 4; ++i) {
		value |= static_cast<uint32_t>(data[i]) << (8 * i);
	}

	return value;
}

float countsToRadians(int32_t counts, int32_t zero_counts, int direction)
{
	return (counts - zero_counts) * direction * (6.28318530717958647692f / 4096.0f);
}

int32_t radiansToCounts(float radians, int32_t zero_counts, int direction)
{
	return zero_counts + static_cast<int32_t>(lroundf(radians * direction * 4096.0f / 6.28318530717958647692f));
}

} // namespace dxl
