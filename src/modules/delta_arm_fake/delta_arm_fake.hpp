#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/delta_arm_state.h>

namespace delta_arm_fake
{

inline delta_arm_state_s zeroState(uint32_t sequence, hrt_abstime timestamp)
{
	delta_arm_state_s state{};
	state.timestamp = timestamp;
	state.timestamp_sample = timestamp;
	state.sequence = sequence;
	state.valid = true;
	return state;
}

class DeltaArmFake : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	DeltaArmFake();
	~DeltaArmFake() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	uORB::Publication<delta_arm_state_s> _delta_arm_state_pub{ORB_ID(delta_arm_state)};
	uint32_t _sequence{0};
};

} // namespace delta_arm_fake
