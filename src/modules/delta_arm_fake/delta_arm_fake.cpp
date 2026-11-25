#include "delta_arm_fake.hpp"

using namespace delta_arm_fake;
using namespace time_literals;

ModuleBase::Descriptor DeltaArmFake::desc{task_spawn, custom_command, print_usage};

DeltaArmFake::DeltaArmFake() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	_delta_arm_state_pub.advertise();
}

bool DeltaArmFake::init()
{
	ScheduleOnInterval(20_ms);
	return true;
}

void DeltaArmFake::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	_delta_arm_state_pub.publish(zeroState(++_sequence, now));
}

int DeltaArmFake::task_spawn(int argc, char *argv[])
{
	DeltaArmFake *instance = new DeltaArmFake();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int DeltaArmFake::print_status()
{
	PX4_INFO_RAW("Published zero joint states: %lu\n", (unsigned long)_sequence);
	return 0;
}

int DeltaArmFake::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DeltaArmFake::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Publish valid zero Delta joint angles for policy testing without arm hardware.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("delta_arm_fake", "test");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int delta_arm_fake_main(int argc, char *argv[])
{
	return ModuleBase::main(DeltaArmFake::desc, argc, argv);
}
