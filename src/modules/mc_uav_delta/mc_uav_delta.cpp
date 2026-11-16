#include "mc_uav_delta.hpp"
#undef OK

#include <rl_tools/persist/backends/tar/operations_generic.h>

#include <sys/stat.h>

ModuleBase::Descriptor UavDelta::desc{task_spawn, custom_command, print_usage};

UavDelta::UavDelta() : ModuleParams(nullptr), ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_actuator_motors_pub.advertise();
	_uav_delta_input_pub.advertise();
	_uav_delta_status_pub.advertise();
}

UavDelta::~UavDelta()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

void UavDelta::reset()
{
	previous_trajectory_setpoint_stale = false;
	timeout_message_sent = false;
	native_step_count = 0;
	intermediate_step_count = 0;

	for (TI i = 0; i < POLICY_ACTION_DIM; ++i) {
		previous_action[i] = 0.0f;
	}

	mc_uav_delta::reset(device, executor, policy, rng);
}

bool UavDelta::test_policy(FILE *file, TI input_offset, TI output_offset)
{
	using POLICY = EXECUTOR_CONFIG::POLICY_TEST;
	POLICY::template Buffer<false> buffers_test;
	POLICY::State<false> policy_state_test;
	rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI,
		 rl_tools::tensor::Shape<TI, 1, POLICY::OUTPUT_SHAPE::LAST>, false>> test_output;
	rl_tools::Mode<rl_tools::mode::Evaluation<>> mode;
	using EXAMPLE_INPUT_SPEC = mc_uav_delta::policy::example::input::SPEC;
	using EXAMPLE_OUTPUT_SPEC = mc_uav_delta::policy::example::output::SPEC;

	float accumulated_difference = 0.0f;
	uint64_t num_values = 0;
	mc_uav_delta::PolicyAction action{};

	for (TI batch_i = 0; batch_i < EXECUTOR_CONFIG::TEST_BATCH_SIZE_ACTUAL; ++batch_i) {
		rl_tools::reset(device, policy, policy_state_test, rng);

		for (TI step_i = 0; step_i < EXECUTOR_CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL; ++step_i) {
			rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI,
				 rl_tools::tensor::Shape<TI, 1, EXAMPLE_INPUT_SPEC::SHAPE::LAST>, false>> batch_input;
			rl_tools::Tensor<rl_tools::tensor::Specification<EXECUTOR_CONFIG::TYPE_POLICY::DEFAULT, TI,
				 rl_tools::tensor::Shape<TI, 1, EXAMPLE_OUTPUT_SPEC::SHAPE::LAST>, false>> batch_output_target;

			fseek(file, input_offset + (step_i * EXAMPLE_INPUT_SPEC::STRIDE::FIRST
						    + batch_i * EXAMPLE_INPUT_SPEC::STRIDE::template GET<1>) * sizeof(EXAMPLE_INPUT_SPEC::T), SEEK_SET);
			fread(batch_input._data, sizeof(EXAMPLE_INPUT_SPEC::T), EXAMPLE_INPUT_SPEC::SHAPE::LAST, file);
			fseek(file, output_offset + (step_i * EXAMPLE_OUTPUT_SPEC::STRIDE::FIRST
						     + batch_i * EXAMPLE_OUTPUT_SPEC::STRIDE::template GET<1>) * sizeof(EXAMPLE_OUTPUT_SPEC::T), SEEK_SET);
			fread(batch_output_target._data, sizeof(EXAMPLE_OUTPUT_SPEC::T), EXAMPLE_OUTPUT_SPEC::SHAPE::LAST, file);

			rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, batch_input), "input is nan");
			rl_tools::evaluate_step(device, policy, batch_input, policy_state_test, test_output, buffers_test, rng, mode);
			rl_tools::utils::assert_exit(device, !rl_tools::is_nan(device, test_output), "output is nan");

			for (TI action_i = 0; action_i < POLICY_ACTION_DIM; ++action_i) {
				accumulated_difference += rl_tools::math::abs(device.math,
							  rl_tools::get(device, test_output, 0, action_i)
							  - rl_tools::get(device, batch_output_target, 0, action_i));
				++num_values;
				rl_tools::utils::assert_exit(device, !rl_tools::math::is_nan(device.math, accumulated_difference),
							     "output is nan");

				if (batch_i == 0 && step_i == EXECUTOR_CONFIG::TEST_SEQUENCE_LENGTH_ACTUAL - 1) {
					action.action[action_i] = rl_tools::get(device, test_output, 0, action_i);
				}
			}
		}
	}

	const float absolute_difference = accumulated_difference / num_values;
	PX4_INFO("Checkpoint test diff: %f", (double)absolute_difference);

	for (TI output_i = 0; output_i < POLICY_ACTION_DIM; ++output_i) {
		PX4_INFO("output[%d]: %f", (int)output_i, (double)action.action[output_i]);
	}

	constexpr float EPSILON = 1e-5f;

	if (absolute_difference >= EPSILON) {
		PX4_ERR("Checkpoint test failed with diff %.10f", (double)absolute_difference);
		return false;
	}

	PX4_INFO("Checkpoint test passed with diff %.10f", (double)absolute_difference);
	return true;
}

bool UavDelta::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity_sub callback registration failed");
		return false;
	}

	const char *path = PX4_STORAGEDIR "/uav_delta/policy.tar";
	struct stat st {};

	if (stat(path, &st) != 0) {
		PX4_ERR("Policy checkpoint %s does not exist", path);
		return false;
	}

	FILE *file = fopen(path, "rb");

	if (file == nullptr) {
		PX4_ERR("Failed to open %s: %s", path, strerror(errno));
		return false;
	}

	if (fseek(file, 0, SEEK_END) != 0) {
		PX4_ERR("fseek failed: %s", strerror(errno));
		fclose(file);
		return false;
	}

	const long size = ftell(file);

	if (size < 0) {
		PX4_ERR("ftell failed: %s", strerror(errno));
		fclose(file);
		return false;
	}

	rewind(file);
	using SPEC = rl_tools::persist::backends::tar::ReaderGroupSpecification<TI,
	      rl_tools::persist::backends::tar::PosixFileData<TI>>;
	rl_tools::persist::backends::tar::ReaderGroup<SPEC> reader_group;
	reader_group.data.f = file;
	reader_group.data.size = size;
	auto actor_group = rl_tools::get_group(device, reader_group, "actor");

	if (!rl_tools::load(device, policy, actor_group)) {
		PX4_ERR("Failed to load policy from file %s", path);
		fclose(file);
		return false;
	}

	constexpr TI METADATA_BUFFER_SIZE = 256;
	char metadata_buffer[METADATA_BUFFER_SIZE];
	TI read_size = 0;
	rl_tools::persist::backends::tar::get(device, reader_group.data, "actor/meta", metadata_buffer,
					      METADATA_BUFFER_SIZE, read_size);
	TI checkpoint_name_position = 0;
	TI checkpoint_name_len = 0;

	if (!rl_tools::persist::backends::tar::seek_in_metadata(device, metadata_buffer, METADATA_BUFFER_SIZE,
			"checkpoint_name", checkpoint_name_position, checkpoint_name_len)) {
		PX4_ERR("Failed to get checkpoint name from metadata");
		fclose(file);
		return false;
	}

	strncpy(checkpoint_name, metadata_buffer + checkpoint_name_position, CHECKPOINT_NAME_LENGTH - 1);
	checkpoint_name[checkpoint_name_len < CHECKPOINT_NAME_LENGTH ? checkpoint_name_len : CHECKPOINT_NAME_LENGTH - 1] = '\0';
	PX4_INFO("Policy loaded from file %s", path);

	TI input_offset = 0;
	TI input_size = 0;
	TI output_offset = 0;
	TI output_size = 0;
	rl_tools::persist::backends::tar::seek(device, reader_group.data, "example/input/data", input_offset, input_size);
	rl_tools::persist::backends::tar::seek(device, reader_group.data, "example/output/data", output_offset, output_size);

	if (!test_policy(file, input_offset, output_offset)) {
		fclose(file);
		return false;
	}

	fclose(file);
	PX4_INFO("Checkpoint name: %s", checkpoint_name);

	int32_t gyro_rate = _param_mc_uavd_gyro_hz.get();

	if (gyro_rate == 0) {
		gyro_rate = _param_imu_gyro_ratemax.get();
	}

	if (gyro_rate <= 0 || gyro_rate % POLICY_CONTROL_FREQUENCY_TRAINING != 0) {
		PX4_ERR("Angular velocity rate %d Hz must be a positive multiple of %d Hz", (int)gyro_rate,
			(int)POLICY_CONTROL_FREQUENCY_TRAINING);
		return false;
	}

	executor.executor.force_sync_native = gyro_rate / POLICY_CONTROL_FREQUENCY_TRAINING;
	executor.executor.force_sync_native_initialized = true;
	PX4_INFO("Angular velocity rate=%d Hz, native GRU rate=%d Hz, ratio=%d", (int)gyro_rate,
		 (int)POLICY_CONTROL_FREQUENCY_TRAINING, (int)executor.executor.force_sync_native);

	register_ext_component_request_s request{};
	request.timestamp = hrt_absolute_time();
	strncpy(request.name, "UAV_DELTA", sizeof(request.name) - 1);
	request.request_id = EXT_COMPONENT_REQUEST_ID;
	request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
	request.register_arming_check = true;
	request.register_mode = true;
	request.request_offboard_setpoints = true;
	_register_ext_component_request_pub.publish(request);

	reset();
	return true;
}

void UavDelta::updateArmingCheckReply()
{
	if (!mode_registered || !_arming_check_request_sub.updated()) {
		return;
	}

	arming_check_request_s request;
	_arming_check_request_sub.copy(&request);

	arming_check_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.request_id = request.request_id;
	reply.registration_id = ext_component_arming_check_id;
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	reply.num_events = 0;
	reply.can_arm_and_run = can_arm;
	reply.mode_req_angular_velocity = true;
	reply.mode_req_local_position = true;
	reply.mode_req_attitude = true;
	reply.mode_req_local_alt = true;
	reply.mode_req_local_position_relaxed = false;
	reply.mode_req_home_position = false;
	reply.mode_req_mission = false;
	reply.mode_req_global_position = false;
	reply.mode_req_global_position_relaxed = false;
	reply.mode_req_prevent_arming = false;
	reply.mode_req_manual_control = false;
	_arming_check_reply_pub.publish(reply);
}

void UavDelta::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();

		if (mode_registered) {
			unregister_ext_component_s request{};
			request.timestamp = hrt_absolute_time();
			strncpy(request.name, "UAV_DELTA", sizeof(request.name) - 1);
			request.arming_check_id = ext_component_arming_check_id;
			request.mode_id = ext_component_mode_id;
			request.mode_executor_id = -1;
			_unregister_ext_component_pub.publish(request);
		}

		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	register_ext_component_reply_s reply;

	if (_register_ext_component_reply_sub.update(&reply)) {
		if (reply.request_id == EXT_COMPONENT_REQUEST_ID && reply.success) {
			ext_component_arming_check_id = reply.arming_check_id;
			ext_component_mode_id = reply.mode_id;
			mode_registered = true;
			PX4_INFO("UAV_DELTA mode registration successful, arming_check_id: %d, mode_id: %d",
				 ext_component_arming_check_id, ext_component_mode_id);
		}
	}

	perf_count(_loop_interval_perf);
	perf_begin(_loop_perf);
	const hrt_abstime current_time = hrt_absolute_time();

	uav_delta_status_s status{};
	status.timestamp = current_time;

	_vehicle_status_sub.update(&_vehicle_status);

	if (_vehicle_status.timestamp > 0) {
		timestamp_last_vehicle_status = current_time;
		timestamp_last_vehicle_status_set = true;
	}

	const bool next_active = timestamp_last_vehicle_status_set && mode_registered
				 && _vehicle_status.nav_state == ext_component_mode_id;
	const bool activated = next_active && !previous_active;

	if (activated) {
		setpoint_config_s config{};
		config.timestamp = hrt_absolute_time();
		config.source_id = ext_component_mode_id;
		config.type = setpoint_config_s::TYPE_DIRECT_ACTUATORS;
		config.should_apply = true;
		_setpoint_config_pub.publish(config);
		reset();
		PX4_INFO("UAV_DELTA activated; recurrent state reset");

	} else if (previous_active && !next_active) {
		PX4_INFO("UAV_DELTA inactive");
	}

	auto finish = [&]() {
		status.active = next_active;
		status.can_arm = can_arm;
		status.timestamp_last_delta_arm_state = timestamp_last_delta_arm_state;
		status.last_delta_sequence = last_delta_sequence;
		status.native_step_count = native_step_count;
		status.intermediate_step_count = intermediate_step_count;
		_uav_delta_status_pub.publish(status);
		updateArmingCheckReply();
		previous_active = next_active;
		perf_end(_loop_perf);
	};

	if (!_vehicle_angular_velocity_sub.update(&_vehicle_angular_velocity)) {
		can_arm = false;
		status.exit_reason = uav_delta_status_s::EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE;
		finish();
		return;
	}

	timestamp_last_angular_velocity = current_time;
	timestamp_last_angular_velocity_set = true;

	if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
		timestamp_last_local_position = current_time;
		timestamp_last_local_position_set = true;
	}

	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
		timestamp_last_attitude = current_time;
		timestamp_last_attitude_set = true;
	}

	trajectory_setpoint_s trajectory_setpoint;

	if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
		bool valid = PX4_ISFINITE(trajectory_setpoint.yaw) && PX4_ISFINITE(trajectory_setpoint.yawspeed);

		for (int i = 0; i < 3; ++i) {
			valid = valid && PX4_ISFINITE(trajectory_setpoint.position[i]) && PX4_ISFINITE(trajectory_setpoint.velocity[i]);
		}

		if (valid) {
			_trajectory_setpoint = trajectory_setpoint;
			timestamp_last_trajectory_setpoint = current_time;
			timestamp_last_trajectory_setpoint_set = true;
		}
	}

	delta_arm_state_s delta_arm_state;

	if (_delta_arm_state_sub.update(&delta_arm_state)) {
		const bool new_sequence = !delta_sequence_set || delta_arm_state.sequence > last_delta_sequence;

		if (new_sequence) {
			last_delta_sequence = delta_arm_state.sequence;
			delta_sequence_set = true;

			if (delta_arm_state.valid && mc_uav_delta::finiteDeltaAngles(delta_arm_state.q_actual)) {
				for (TI i = 0; i < DELTA_OBSERVATION_DIM; ++i) {
					q_actual[i] = delta_arm_state.q_actual[i];
				}

				timestamp_last_delta_arm_state = current_time;
				delta_stream_valid = true;

			} else {
				delta_stream_valid = false;
			}
		}
	}

	const hrt_abstime delta_max_age = (hrt_abstime)_param_mc_uavd_q_maxage.get() * 1000;
	status.delta_arm_state_stale = mc_uav_delta::deltaStateIsStale(delta_sequence_set, delta_stream_valid,
				       current_time, timestamp_last_delta_arm_state, delta_max_age);

	if (!timestamp_last_angular_velocity_set || !timestamp_last_local_position_set || !timestamp_last_attitude_set
	    || !delta_sequence_set) {
		can_arm = false;
		status.exit_reason = uav_delta_status_s::EXIT_REASON_NOT_ALL_OBSERVATIONS_SET;
		status.vehicle_angular_velocity_stale = !timestamp_last_angular_velocity_set;
		status.vehicle_local_position_stale = !timestamp_last_local_position_set;
		status.vehicle_attitude_stale = !timestamp_last_attitude_set;
		status.delta_arm_state_stale = !delta_sequence_set || status.delta_arm_state_stale;
		finish();
		return;
	}

	if ((current_time - timestamp_last_angular_velocity) > OBSERVATION_TIMEOUT_ANGULAR_VELOCITY) {
		can_arm = false;
		status.exit_reason = uav_delta_status_s::EXIT_REASON_ANGULAR_VELOCITY_STALE;
		status.vehicle_angular_velocity_stale = true;
		finish();
		return;
	}

	if ((current_time - timestamp_last_local_position) > OBSERVATION_TIMEOUT_LOCAL_POSITION) {
		can_arm = false;
		status.exit_reason = uav_delta_status_s::EXIT_REASON_LOCAL_POSITION_STALE;
		status.vehicle_local_position_stale = true;
		finish();
		return;
	}

	if ((current_time - timestamp_last_attitude) > OBSERVATION_TIMEOUT_ATTITUDE) {
		can_arm = false;
		status.exit_reason = uav_delta_status_s::EXIT_REASON_ATTITUDE_STALE;
		status.vehicle_attitude_stale = true;
		finish();
		return;
	}

	if (status.delta_arm_state_stale) {
		can_arm = false;
		status.exit_reason = uav_delta_status_s::EXIT_REASON_DELTA_ARM_STATE_STALE;
		finish();
		return;
	}

	can_arm = true;
	timeout_message_sent = false;

	if (!timestamp_last_trajectory_setpoint_set
	    || (current_time - timestamp_last_trajectory_setpoint) > TRAJECTORY_SETPOINT_TIMEOUT) {
		status.trajectory_setpoint_stale = true;

		if (!previous_trajectory_setpoint_stale || activated) {
			_trajectory_setpoint.position[0] = _vehicle_local_position.x;
			_trajectory_setpoint.position[1] = _vehicle_local_position.y;
			_trajectory_setpoint.position[2] = _vehicle_local_position.z;
			_trajectory_setpoint.velocity[0] = 0;
			_trajectory_setpoint.velocity[1] = 0;
			_trajectory_setpoint.velocity[2] = 0;
			_trajectory_setpoint.yawspeed = 0;

			const auto &q = _vehicle_attitude.q;
			_trajectory_setpoint.yaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]),
							  1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
		}

		previous_trajectory_setpoint_stale = true;

	} else {
		status.trajectory_setpoint_stale = false;
		previous_trajectory_setpoint_stale = false;
	}

	mc_uav_delta::UavState state{};
	state.position[0] = _vehicle_local_position.x;
	state.position[1] = _vehicle_local_position.y;
	state.position[2] = _vehicle_local_position.z;
	state.linear_velocity[0] = _vehicle_local_position.vx;
	state.linear_velocity[1] = _vehicle_local_position.vy;
	state.linear_velocity[2] = _vehicle_local_position.vz;

	for (TI i = 0; i < 4; ++i) {
		state.attitude[i] = _vehicle_attitude.q[i];
	}

	for (TI i = 0; i < 3; ++i) {
		state.angular_velocity[i] = _vehicle_angular_velocity.xyz[i];
	}

	mc_uav_delta::UavTarget target{};

	for (TI i = 0; i < 3; ++i) {
		target.position[i] = _trajectory_setpoint.position[i];
		target.linear_velocity[i] = _trajectory_setpoint.velocity[i];
	}

	target.yaw = _trajectory_setpoint.yaw;
	auto observation = mc_uav_delta::buildPolicyObservation(state, target, previous_action, q_actual);
	mc_uav_delta::PolicyAction action{};
	const hrt_abstime nanoseconds = current_time * 1000;
	auto executor_status = mc_uav_delta::control(device, executor, nanoseconds, policy, observation, action, rng);

	if (!executor_status.OK) {
		if (executor_status.TIMESTAMP_INVALID) {
			PX4_ERR("RLtools executor error: Timestamp invalid");
		}

		if (executor_status.LAST_CONTROL_TIMESTAMP_GREATER_THAN_LAST_OBSERVATION_TIMESTAMP) {
			PX4_ERR("RLtools executor error: Control timestamp exceeds observation timestamp");
		}
	}

	if (executor_status.source != decltype(executor_status.source)::CONTROL) {
		finish();
		return;
	}

	if (executor_status.step_type == decltype(executor_status.step_type)::NATIVE) {
		++native_step_count;

	} else {
		++intermediate_step_count;
	}

	uav_delta_input_s input{};
	input.timestamp = current_time;
	input.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;
	input.active = next_active;
	input.native_step = executor_status.step_type == decltype(executor_status.step_type)::NATIVE;

	for (TI i = 0; i < POLICY_OBSERVATION_DIM; ++i) {
		input.observation[i] = rl_tools::get(device, executor.executor.observation, 0, i);
	}

	for (TI i = 0; i < POLICY_ACTION_DIM; ++i) {
		input.action[i] = action.action[i];
		previous_action[i] = action.action[i];
	}

	_uav_delta_input_pub.publish(input);

	actuator_motors_s actuator_motors{};
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _vehicle_angular_velocity.timestamp_sample;

	for (TI i = 0; i < actuator_motors_s::NUM_CONTROLS; ++i) {
		if (i < POLICY_ACTION_DIM) {
			actuator_motors.control[i] = (action.action[i] + 1.0f) / 2.0f;

		} else {
			actuator_motors.control[i] = NAN;
		}
	}

	actuator_motors_s remapped = actuator_motors;
	remapped.control[0] = actuator_motors.control[0];
	remapped.control[1] = actuator_motors.control[2];
	remapped.control[2] = actuator_motors.control[3];
	remapped.control[3] = actuator_motors.control[1];

	if (next_active) {
		_actuator_motors_pub.publish(remapped);
	}

	finish();
}

int UavDelta::task_spawn(int argc, char *argv[])
{
	UavDelta *instance = new UavDelta();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	instance->_vehicle_angular_velocity_sub.unregisterCallback();
	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int UavDelta::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	PX4_INFO_RAW("Checkpoint: %s\n", checkpoint_name);
	PX4_INFO_RAW("Native steps: %lu, intermediate steps: %lu\n", (unsigned long)native_step_count,
		     (unsigned long)intermediate_step_count);
	PX4_INFO_RAW("Delta sequence: %lu, valid: %s\n", (unsigned long)last_delta_sequence,
		     delta_stream_valid ? "yes" : "no");
	return 0;
}

int UavDelta::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UavDelta::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
UAV Delta recurrent flight mode with RAPTOR 22D observations and three raw Delta joint angles.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_uav_delta", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int mc_uav_delta_main(int argc, char *argv[])
{
	return ModuleBase::main(UavDelta::desc, argc, argv);
}
