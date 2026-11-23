#pragma once

#include <stdint.h>

namespace dxl
{
inline constexpr uint32_t RANDOM_TRAJECTORY_SAMPLE_COUNT = 3001;
inline constexpr uint32_t RANDOM_TRAJECTORY_PERIOD_US = 20000;
inline constexpr uint32_t RANDOM_TRAJECTORY_DURATION_US = 60000000;
inline constexpr float RANDOM_TRAJECTORY_HOME_TOLERANCE_RAD = 0.15f;
extern const float RANDOM_TRAJECTORY_Q[RANDOM_TRAJECTORY_SAMPLE_COUNT][3];
} // namespace dxl
