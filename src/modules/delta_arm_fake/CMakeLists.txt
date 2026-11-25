include(px4_add_library)

px4_add_module(
	MODULE modules__delta_arm_fake
	MAIN delta_arm_fake
	STACK_MAIN 2000
	SRCS
		delta_arm_fake.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
)

px4_add_unit_gtest(
	SRC DeltaArmFakeTest.cpp
	LINKLIBS modules__delta_arm_fake
)
