include(px4_add_library)

px4_add_module(
	MODULE drivers__dxl_arm
	MAIN dxl_arm
	STACK_MAIN 4000
	SRCS
		dxl_arm.cpp
		dxl_protocol.cpp
		dxl_motion.cpp
		dxl_rc.cpp
	MODULE_CONFIG
		module.yaml
)

target_compile_features(drivers__dxl_arm PRIVATE cxx_std_17)

px4_add_unit_gtest(
	SRC DxlArmTest.cpp
	EXTRA_SRCS dxl_protocol.cpp dxl_motion.cpp dxl_rc.cpp
)
