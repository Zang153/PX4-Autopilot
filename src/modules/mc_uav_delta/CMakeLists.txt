include(px4_add_library)

px4_add_module(
	MODULE modules__mc_uav_delta
	MAIN mc_uav_delta
	STACK_MAIN 4000
	SRCS
		mc_uav_delta.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
		rl_tools
)

target_compile_features(modules__mc_uav_delta PRIVATE cxx_std_17)
target_compile_options(modules__mc_uav_delta PRIVATE -Wno-unused-result)

px4_add_unit_gtest(
	SRC PolicyObservationTest.cpp
	LINKLIBS modules__mc_uav_delta rl_tools
)
