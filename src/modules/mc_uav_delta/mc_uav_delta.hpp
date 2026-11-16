#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/delta_arm_state.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/setpoint_config.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/uav_delta_input.h>
#include <uORB/topics/uav_delta_status.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#undef OK

#ifdef __PX4_POSIX
#include <rl_tools/operations/cpu.h>
#else
#include <rl_tools/operations/arm.h>
#endif

#include <rl_tools/nn/layers/standardize/operations_generic.h>
#include <rl_tools/nn/layers/dense/operations_arm/opt.h>
#include <rl_tools/nn/layers/sample_and_squash/operations_generic.h>
#include <rl_tools/nn/layers/gru/operations_generic.h>
#include <rl_tools/nn_models/mlp/operations_generic.h>
#include <rl_tools/nn_models/sequential/operations_generic.h>

#include <rl_tools/persist/backends/tar/operations_posix.h>
#include <rl_tools/nn/optimizers/adam/instance/persist.h>
#include <rl_tools/nn/layers/gru/persist.h>
#include <rl_tools/nn/layers/dense/persist.h>
#include <rl_tools/nn_models/sequential/persist.h>

#include "policy/executor.hpp"
#include "policy/policy.hpp"

using namespace mc_uav_delta;
using namespace time_literals;

class UavDelta : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	UavDelta();
	~UavDelta() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
#ifdef __PX4_POSIX
	using DEVICE = rl_tools::devices::DefaultCPU;
#else
	using DEV_SPEC = rl_tools::devices::DefaultARMSpecification;
	using DEVICE = rl_tools::devices::arm::OPT<DEV_SPEC>;
#endif
	using TI = typename DEVICE::index_t;
	using RNG = typename DEVICE::SPEC::RANDOM::ENGINE<>;
	using T = float;

	void Run() override;
	void reset();
	bool test_policy(FILE *file, TI input_offset, TI output_offset);
	void updateArmingCheckReply();

	static constexpr uint64_t EXT_COMPONENT_REQUEST_ID = 424242;
	static constexpr TI OBSERVATION_TIMEOUT_ANGULAR_VELOCITY = 10 * 1000;
	static constexpr TI OBSERVATION_TIMEOUT_LOCAL_POSITION = 100 * 1000;
	static constexpr TI OBSERVATION_TIMEOUT_ATTITUDE = 50 * 1000;
	static constexpr TI TRAJECTORY_SETPOINT_TIMEOUT = 200 * 1000;
	static constexpr TI POLICY_CONTROL_FREQUENCY_TRAINING = 100;

	DEVICE device;
	RNG rng;

	decltype(register_ext_component_reply_s::mode_id) ext_component_mode_id{-1};
	decltype(register_ext_component_reply_s::arming_check_id) ext_component_arming_check_id{-1};
	bool mode_registered{false};
	bool can_arm{false};
	bool previous_active{false};
	bool previous_trajectory_setpoint_stale{false};
	bool timeout_message_sent{false};

	vehicle_local_position_s _vehicle_local_position{};
	vehicle_angular_velocity_s _vehicle_angular_velocity{};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_status_s _vehicle_status{};
	trajectory_setpoint_s _trajectory_setpoint{};

	hrt_abstime timestamp_last_local_position{0};
	hrt_abstime timestamp_last_angular_velocity{0};
	hrt_abstime timestamp_last_attitude{0};
	hrt_abstime timestamp_last_trajectory_setpoint{0};
	hrt_abstime timestamp_last_vehicle_status{0};
	bool timestamp_last_local_position_set{false};
	bool timestamp_last_angular_velocity_set{false};
	bool timestamp_last_attitude_set{false};
	bool timestamp_last_trajectory_setpoint_set{false};
	bool timestamp_last_vehicle_status_set{false};

	float q_actual[DELTA_OBSERVATION_DIM] {};
	hrt_abstime timestamp_last_delta_arm_state{0};
	uint32_t last_delta_sequence{0};
	bool delta_sequence_set{false};
	bool delta_stream_valid{false};

	T previous_action[POLICY_ACTION_DIM] {};

	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _delta_arm_state_sub{ORB_ID(delta_arm_state)};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<uav_delta_input_s> _uav_delta_input_pub{ORB_ID(uav_delta_input)};
	uORB::Publication<uav_delta_status_s> _uav_delta_status_pub{ORB_ID(uav_delta_status)};
	uORB::Publication<register_ext_component_request_s> _register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<setpoint_config_s> _setpoint_config_pub{ORB_ID(setpoint_config)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	struct EXECUTOR_CONFIG {
		static constexpr TI CONTROL_INTERVAL_INTERMEDIATE_NS = 2.5 * 1000 * 1000;
		static constexpr TI CONTROL_INTERVAL_NATIVE_NS = 10 * 1000 * 1000;
		static constexpr bool FORCE_SYNC_INTERMEDIATE = true;
		static constexpr bool FORCE_SYNC_NATIVE_RUNTIME = true;
		static constexpr TI FORCE_SYNC_NATIVE = 4;
		static constexpr bool DYNAMIC_ALLOCATION = false;

		using ACTOR_TYPE_ORIGINAL = mc_uav_delta::policy::TYPE;
		using POLICY_TEST = ACTOR_TYPE_ORIGINAL::template CHANGE_BATCH_SIZE<TI, 1>::template CHANGE_SEQUENCE_LENGTH<TI, 1>;
		using POLICY_BATCH_SIZE = ACTOR_TYPE_ORIGINAL::template CHANGE_BATCH_SIZE<TI, 1>;
		using POLICY = typename POLICY_BATCH_SIZE::template CHANGE_CAPABILITY<rl_tools::nn::capability::Forward<false, false>>;
		using TYPE_POLICY = typename POLICY::TYPE_POLICY;

		struct WARNING_LEVELS : rl_tools::inference::executor::WarningLevelsDefault<TYPE_POLICY> {
			using T = typename TYPE_POLICY::DEFAULT;
			static constexpr T INTERMEDIATE_TIMING_JITTER_HIGH_THRESHOLD = 2.0;
			static constexpr T INTERMEDIATE_TIMING_JITTER_LOW_THRESHOLD = 0.5;
			static constexpr T INTERMEDIATE_TIMING_BIAS_HIGH_THRESHOLD = 2.0;
			static constexpr T INTERMEDIATE_TIMING_BIAS_LOW_THRESHOLD = 0.5;
			static constexpr T NATIVE_TIMING_JITTER_HIGH_THRESHOLD = 2.0;
			static constexpr T NATIVE_TIMING_JITTER_LOW_THRESHOLD = 0.5;
			static constexpr T NATIVE_TIMING_BIAS_HIGH_THRESHOLD = 2.0;
			static constexpr T NATIVE_TIMING_BIAS_LOW_THRESHOLD = 0.5;
		};

		using TIMESTAMP = hrt_abstime;
		static constexpr TI TEST_SEQUENCE_LENGTH_ACTUAL = 5;
		static constexpr TI TEST_BATCH_SIZE_ACTUAL = 2;

		using EXECUTOR_SPEC = mc_uav_delta::PolicyExecutorSpecification<TYPE_POLICY, TI, TIMESTAMP, POLICY,
		      CONTROL_INTERVAL_INTERMEDIATE_NS, CONTROL_INTERVAL_NATIVE_NS, FORCE_SYNC_INTERMEDIATE,
		      FORCE_SYNC_NATIVE, FORCE_SYNC_NATIVE_RUNTIME, WARNING_LEVELS, DYNAMIC_ALLOCATION>;
	};

	using EXECUTOR_SPEC = EXECUTOR_CONFIG::EXECUTOR_SPEC;
	mc_uav_delta::PolicyExecutor<EXECUTOR_SPEC> executor;
	EXECUTOR_CONFIG::POLICY policy;

	static constexpr TI CHECKPOINT_NAME_LENGTH = 100;
	char checkpoint_name[CHECKPOINT_NAME_LENGTH] = "n/a";
	uint32_t native_step_count{0};
	uint32_t intermediate_step_count{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::IMU_GYRO_RATEMAX>) _param_imu_gyro_ratemax,
		(ParamInt<px4::params::MC_UAVD_GYRO_HZ>) _param_mc_uavd_gyro_hz,
		(ParamInt<px4::params::MC_UAVD_Q_MAXAGE>) _param_mc_uavd_q_maxage
	)
};
