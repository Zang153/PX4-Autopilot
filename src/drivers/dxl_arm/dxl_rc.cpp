#include "dxl_rc.hpp"

namespace dxl
{

RcMode rcModeFromChannel(uint16_t value)
{
	if (value >= 1900 && value <= 2100) {
		return RcMode::UP;
	}

	if (value >= 1403 && value <= 1603) {
		return RcMode::CENTER;
	}

	if (value >= 899 && value <= 1099) {
		return RcMode::DOWN;
	}

	return RcMode::UNKNOWN;
}

bool angleForMode(RcMode mode, float minimum, float maximum, float &angle)
{
	switch (mode) {
	case RcMode::UP:
		angle = minimum;
		return true;

	case RcMode::CENTER:
		angle = 0.0f;
		return true;

	case RcMode::DOWN:
		angle = maximum;
		return true;

	case RcMode::UNKNOWN:
		return false;
	}

	return false;
}

} // namespace dxl
