#pragma once

#include <stdint.h>

namespace dxl
{

enum class RcMode : uint8_t {
	UNKNOWN = 0,
	UP = 1,
	CENTER = 2,
	DOWN = 3,
};

RcMode rcModeFromChannel(uint16_t value);

class RcSwitch
{
public:
	RcMode update(RcMode raw_mode, bool signal_valid, uint64_t timestamp_us, uint32_t debounce_us);

private:
	RcMode _candidate{RcMode::UNKNOWN};
	RcMode _position{RcMode::UNKNOWN};
	uint64_t _candidate_since{0};
};

} // namespace dxl
