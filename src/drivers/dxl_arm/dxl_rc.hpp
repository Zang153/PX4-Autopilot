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
bool angleForMode(RcMode mode, float minimum, float maximum, float &angle);

} // namespace dxl
