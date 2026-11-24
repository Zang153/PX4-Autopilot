#pragma once

#include "dxl_random_trajectory.hpp"

#include <stdint.h>

namespace dxl
{

class RandomTrajectory
{
public:
	bool start(uint64_t timestamp_us);
	void stop() { _active = false; }
	bool active() const { return _active; }
	uint32_t sampleIndex() const { return _sample_index; }
	bool update(uint64_t timestamp_us, float q_target[3]);

private:
	bool _active{false};
	uint64_t _start_time{0};
	uint32_t _sample_index{0};
};

} // namespace dxl
