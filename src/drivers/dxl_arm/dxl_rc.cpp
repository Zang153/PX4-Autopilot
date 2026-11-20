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

RcMode RcSwitch::update(RcMode raw_mode, bool signal_valid, uint64_t timestamp_us, uint32_t debounce_us)
{
	if (!signal_valid) {
		_candidate = RcMode::UNKNOWN;
		_position = RcMode::UNKNOWN;
		_candidate_since = 0;
		return _position;
	}

	if (raw_mode != _candidate || timestamp_us < _candidate_since) {
		_candidate = raw_mode;
		_candidate_since = timestamp_us;
	}

	if (_position != _candidate && timestamp_us - _candidate_since >= debounce_us) {
		_position = _candidate;
	}

	return _position;
}

} // namespace dxl
