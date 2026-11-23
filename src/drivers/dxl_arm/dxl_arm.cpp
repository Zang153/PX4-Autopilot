#include "dxl_arm.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

ModuleBase::Descriptor DxlArm::desc{task_spawn, custom_command, print_usage};

DxlArm::DxlArm() : ModuleParams(nullptr)
{
	_delta_arm_state_pub.advertise();
	_dxl_arm_status_pub.advertise();
}

DxlArm::~DxlArm()
{
	closePort();
}

int DxlArm::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int, char **) -> ModuleBase * {
		return new DxlArm();
	}, argc, argv);
}

int DxlArm::task_spawn(int argc, char *argv[])
{
	const int task_id = px4_task_spawn_cmd("dxl_arm", SCHED_DEFAULT, SCHED_PRIORITY_SLOW_DRIVER, 4000,
					       (px4_main_t)&run_trampoline, argv);

	if (task_id < 0) {
		desc.task_id = -1;
		PX4_ERR("Failed to spawn task");
		return PX4_ERROR;
	}

	desc.task_id = task_id;
	return PX4_OK;
}

int DxlArm::custom_command(int argc, char *argv[])
{
	DxlArm *instance = get_instance<DxlArm>(desc);

	if (instance == nullptr) {
		PX4_ERR("dxl_arm is not running");
		return PX4_ERROR;
	}

	if (strcmp(argv[0], "center") == 0) {
		instance->_requested_command.store(Command::CENTER);
		return PX4_OK;
	}

	if (strcmp(argv[0], "raise") == 0) {
		instance->_requested_command.store(Command::RAISE);
		return PX4_OK;
	}

	if (strcmp(argv[0], "down") == 0) {
		instance->_requested_command.store(Command::DOWN);
		return PX4_OK;
	}

	if (strcmp(argv[0], "random") == 0) {
		instance->_requested_command.store(Command::RANDOM);
		return PX4_OK;
	}

	if (strcmp(argv[0], "hold") == 0) {
		instance->_requested_command.store(Command::HOLD);
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int DxlArm::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Dynamixel Protocol 2.0 driver and joint-space Delta arm motion controller for the 7-Nano TELEM1 port.
Feedback is published as delta_arm_state in the training joint convention. CH7 down starts the fixed 60 s random trajectory from home.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dxl_arm", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("center", "Write zero angle directly");
	PRINT_MODULE_USAGE_COMMAND_DESCR("raise", "Write the configured upward angle directly");
	PRINT_MODULE_USAGE_COMMAND_DESCR("down", "Write the configured downward angle directly");
	PRINT_MODULE_USAGE_COMMAND_DESCR("random", "Play the fixed 60 s trajectory once from home");
	PRINT_MODULE_USAGE_COMMAND_DESCR("hold", "Stop the trajectory and keep the current target");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return PX4_OK;
}

bool DxlArm::validConfiguration() const
{
	const int id1 = _param_id1.get();
	const int id2 = _param_id2.get();
	const int id3 = _param_id3.get();

	if (id1 < 1 || id1 > 253 || id2 < 1 || id2 > 253 || id3 < 1 || id3 > 253
	    || id1 == id2 || id1 == id3 || id2 == id3) {
		return false;
	}

	if (direction(0) * direction(0) != 1 || direction(1) * direction(1) != 1
	    || direction(2) * direction(2) != 1) {
		return false;
	}

	return _param_q_min.get() < 0.0f && _param_q_max.get() > 0.0f
	       && _param_feedback_rate.get() > 0 && _param_command_rate.get() > 0;
}

int DxlArm::servoId(uint8_t index) const
{
	return index == 0 ? _param_id1.get() : index == 1 ? _param_id2.get() : _param_id3.get();
}

int DxlArm::zeroCount(uint8_t index) const
{
	return index == 0 ? _param_zero1.get() : index == 1 ? _param_zero2.get() : _param_zero3.get();
}

int DxlArm::direction(uint8_t index) const
{
	return index == 0 ? _param_sign1.get() : index == 1 ? _param_sign2.get() : _param_sign3.get();
}

bool DxlArm::openPort()
{
	if (!validConfiguration()) {
		PX4_ERR("Invalid IDs, directions, joint limits or rates");
		return false;
	}

	_fd = ::open("/dev/ttyS5", O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("Failed to open /dev/ttyS5 (errno=%d)", errno);
		return false;
	}

	termios options{};

	if (tcgetattr(_fd, &options) != 0) {
		PX4_ERR("tcgetattr failed");
		closePort();
		return false;
	}

	options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF);
	options.c_oflag &= ~OPOST;
	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	options.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
	options.c_cflag |= CS8 | CREAD | CLOCAL;

	speed_t speed;

	switch (_param_baud.get()) {
	case 57600: speed = B57600; break;
	case 115200: speed = B115200; break;
	case 1000000: speed = B1000000; break;
	default:
		PX4_ERR("Unsupported baud rate %d", (int)_param_baud.get());
		closePort();
		return false;
	}

	if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0
	    || tcsetattr(_fd, TCSANOW, &options) != 0) {
		PX4_ERR("Failed to configure serial port");
		closePort();
		return false;
	}

	tcflush(_fd, TCIOFLUSH);
	return true;
}

void DxlArm::closePort()
{
	if (_fd >= 0) {
		::close(_fd);
		_fd = -1;
	}
}

bool DxlArm::writeAll(const uint8_t *data, size_t length, uint32_t timeout_us)
{
	size_t written = 0;
	hrt_abstime start = hrt_absolute_time();

	while (written < length) {
		const ssize_t result = ::write(_fd, data + written, length - written);

		if (result > 0) {
			written += result;
			continue;
		}

		if (result < 0 && errno == EINTR) {
			continue;
		}

		if (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			return false;
		}

		const uint64_t elapsed = hrt_elapsed_time(&start);

		if (elapsed >= timeout_us) {
			return false;
		}

		pollfd pfd{.fd = _fd, .events = POLLOUT, .revents = 0};
		const int remaining_ms = static_cast<int>((timeout_us - elapsed + 999) / 1000);

		if (::poll(&pfd, 1, remaining_ms) < 0 && errno != EINTR) {
			return false;
		}
	}

	return true;
}

int DxlArm::exchange(const uint8_t *request, size_t request_length, dxl::StatusPacket replies[3],
		     uint32_t timeout_us)
{
	tcflush(_fd, TCIFLUSH);

	if (!writeAll(request, request_length, timeout_us)) {
		return -1;
	}

	uint8_t buffer[256] {};
	size_t total = 0;
	int count = 0;
	hrt_abstime start = hrt_absolute_time();

	while (hrt_elapsed_time(&start) < timeout_us && total < sizeof(buffer)) {
		const uint64_t elapsed = hrt_elapsed_time(&start);
		pollfd pfd{.fd = _fd, .events = POLLIN, .revents = 0};
		const int remaining_ms = static_cast<int>((timeout_us - elapsed + 999) / 1000);
		const int result = ::poll(&pfd, 1, remaining_ms);

		if (result < 0) {
			if (errno == EINTR) {
				continue;
			}

			return -1;
		}

		if (result == 0 || !(pfd.revents & POLLIN)) {
			continue;
		}

		const ssize_t bytes = ::read(_fd, buffer + total, sizeof(buffer) - total);

		if (bytes < 0) {
			if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
				continue;
			}

			return -1;
		}

		if (bytes == 0) {
			continue;
		}

		total += bytes;
		size_t offset = 0;
		dxl::StatusPacket parsed[3] {};
		const size_t parsed_count = dxl::parseStatusPackets(buffer, total, parsed, 3, offset);

		for (size_t i = 0; i < parsed_count; ++i) {
			bool duplicate = false;

			for (int j = 0; j < count; ++j) {
				duplicate |= replies[j].id == parsed[i].id;
			}

			if (!duplicate && count < 3) {
				replies[count++] = parsed[i];
			}
		}

		if (offset > 0) {
			memmove(buffer, buffer + offset, total - offset);
			total -= offset;
		}

		if (count == 3) {
			return count;
		}
	}

	return count;
}

bool DxlArm::readFeedback()
{
	uint8_t ids[3];

	for (uint8_t i = 0; i < 3; ++i) {
		ids[i] = static_cast<uint8_t>(servoId(i));
	}

	uint8_t request[32] {};
	const size_t request_length = dxl::buildSyncRead(ids, dxl::PRESENT_POSITION_ADDRESS, 4,
				      request, sizeof(request));
	const hrt_abstime start = hrt_absolute_time();
	dxl::StatusPacket replies[3] {};
	const int count = exchange(request, request_length, replies, 30000);
	_read_latency_us = hrt_elapsed_time(&start);
	bool valid = count == 3;
	int32_t raw[3] {};
	uint8_t errors[3] {};
	bool seen[3] {};

	for (int i = 0; i < count; ++i) {
		for (uint8_t joint = 0; joint < 3; ++joint) {
			if (replies[i].id == ids[joint]) {
				seen[joint] = replies[i].param_len == 4;
				errors[joint] = replies[i].error;
				raw[joint] = static_cast<int32_t>(dxl::decodeUint32(replies[i].params, 4));
				valid &= seen[joint] && errors[joint] == 0 && raw[joint] >= 0 && raw[joint] <= 4095;
			}
		}
	}

	for (uint8_t i = 0; i < 3; ++i) {
		valid &= seen[i];
	}

	const hrt_abstime now = hrt_absolute_time();

	if (!valid) {
		++_read_errors;
		_feedback_valid = false;
		_torque_ready = false;
		return false;
	}

	if (_last_valid_feedback > 0) {
		_feedback_period_us = now - _last_valid_feedback;
	}

	for (uint8_t i = 0; i < 3; ++i) {
		_raw_positions[i] = raw[i];
		_dxl_errors[i] = errors[i];
		_q_actual[i] = dxl::countsToRadians(raw[i], zeroCount(i), direction(i));
	}

	_feedback_valid = true;
	_last_valid_feedback = now;
	return true;
}

bool DxlArm::writeGoals(const float positions[3])
{
	uint8_t ids[3];
	int32_t raw[3];

	for (uint8_t i = 0; i < 3; ++i) {
		ids[i] = servoId(i);
		const float limited = fminf(_param_q_max.get(), fmaxf(_param_q_min.get(), positions[i]));
		raw[i] = dxl::radiansToCounts(limited, zeroCount(i), direction(i));
		raw[i] = raw[i] < 0 ? 0 : raw[i] > 4095 ? 4095 : raw[i];
	}

	uint8_t request[48] {};
	const size_t length = dxl::buildSyncWritePositions(ids, raw, request, sizeof(request));

	if (!writeAll(request, length, 30000)) {
		++_write_errors;
		_torque_ready = false;
		return false;
	}

	return true;
}

bool DxlArm::setTorque(bool enabled)
{
	uint8_t ids[3];

	for (uint8_t i = 0; i < 3; ++i) {
		ids[i] = servoId(i);
	}

	uint8_t request[32] {};
	const size_t length = dxl::buildSyncWriteTorque(ids, enabled, request, sizeof(request));
	return writeAll(request, length, 30000);
}

bool DxlArm::ensureTorque()
{
	if (_torque_ready) {
		return true;
	}

	if (!setTorque(true)) {
		++_write_errors;
		_torque_ready = false;
		return false;
	}

	_torque_ready = true;
	return true;
}

void DxlArm::publishFeedback(bool valid, hrt_abstime timestamp)
{
	delta_arm_state_s state{};
	state.timestamp = hrt_absolute_time();
	state.timestamp_sample = timestamp;
	state.sequence = ++_feedback_sequence;
	state.valid = valid;

	if (valid) {
		for (uint8_t i = 0; i < 3; ++i) {
			state.q_actual[i] = _q_actual[i];
		}
	}

	_delta_arm_state_pub.publish(state);
}

void DxlArm::publishStatus(hrt_abstime timestamp)
{
	dxl_arm_status_s status{};
	status.timestamp = timestamp;
	status.state = _random_trajectory.active() ? dxl_arm_status_s::STATE_RANDOM : static_cast<uint8_t>(_command_mode);
	status.torque_initialized = _torque_ready;
	status.feedback_valid = _feedback_valid;
	status.trajectory_active = _random_trajectory.active();
	status.trajectory_sample = _random_trajectory.sampleIndex();
	status.rc_mode = static_cast<uint8_t>(_rc_mode);
	status.feedback_sequence = _feedback_sequence;
	status.feedback_period_us = _feedback_period_us;
	status.read_latency_us = _read_latency_us;
	status.read_errors = _read_errors;
	status.write_errors = _write_errors;

	for (uint8_t i = 0; i < 3; ++i) {
		status.raw_position[i] = _raw_positions[i];
		status.q_actual[i] = _q_actual[i];
		status.q_target[i] = _q_target[i];
		status.dxl_error[i] = _dxl_errors[i];
	}

	_dxl_arm_status_pub.publish(status);
}

void DxlArm::updateRc(hrt_abstime timestamp)
{
	auto invalidate = [&]() {
		if (_random_trajectory.active()) {
			_random_trajectory.stop();
			_command_mode = dxl::RcMode::UNKNOWN;
			PX4_WARN("Random trajectory stopped; holding the last target");
		}

		_rc_mode = dxl::RcMode::UNKNOWN;
	};

	if (_param_rc_channel.get() <= 0) {
		invalidate();
		return;
	}

	if (_input_rc_sub.update(&_input_rc)) {
		_input_rc_seen = true;
	}

	if (!_input_rc_seen || timestamp - _input_rc.timestamp_last_signal >= 250000
	    || _input_rc.rc_lost || _input_rc.rc_failsafe) {
		invalidate();
		return;
	}

	const int channel = _param_rc_channel.get();

	if (channel <= 0 || channel > _input_rc.channel_count || channel > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		invalidate();
		return;
	}

	const dxl::RcMode mode = dxl::rcModeFromChannel(_input_rc.values[channel - 1]);

	if (mode == dxl::RcMode::UNKNOWN) {
		invalidate();
		return;
	}

	if (mode == _rc_mode) {
		return;
	}

	_rc_mode = mode;

	switch (mode) {
	case dxl::RcMode::CENTER:
		requestCommand(Command::CENTER);
		break;

	case dxl::RcMode::UP:
		requestCommand(Command::RAISE);
		break;

	case dxl::RcMode::DOWN:
		requestCommand(Command::RANDOM);
		break;

	case dxl::RcMode::UNKNOWN:
		break;
	}
}

bool DxlArm::requestCommand(Command command)
{
	if (command == Command::RANDOM) {
		if (!dxl::randomTrajectoryStartReady(_feedback_valid, _q_actual)) {
			PX4_WARN("Random trajectory requires valid home feedback; center the arm first");
			return requestCommand(Command::CENTER);
		}

		if (!_random_trajectory.start(hrt_absolute_time())) {
			return false;
		}

		for (uint8_t i = 0; i < 3; ++i) {
			_q_target[i] = dxl::RANDOM_TRAJECTORY_Q[0][i];
		}

		_command_mode = dxl::RcMode::DOWN;
		_torque_requested = true;
		_next_command = 0;
		PX4_INFO("Random trajectory started: 3001 samples at 50 Hz");
		return true;
	}

	if (_random_trajectory.active()) {
		_random_trajectory.stop();
		PX4_INFO("Random trajectory stopped");
	}

	if (command == Command::HOLD) {
		_command_mode = dxl::RcMode::UNKNOWN;
		_torque_requested = true;
		return true;
	}

	dxl::RcMode mode = dxl::RcMode::UNKNOWN;

	switch (command) {
	case Command::CENTER:
		mode = dxl::RcMode::CENTER;
		break;

	case Command::RAISE:
		mode = dxl::RcMode::UP;
		break;

	case Command::DOWN:
		mode = dxl::RcMode::DOWN;
		break;

	default:
		return false;
	}

	float angle = 0.0f;

	if (!dxl::angleForMode(mode, _param_q_min.get(), _param_q_max.get(), angle)) {
		return false;
	}

	for (uint8_t i = 0; i < 3; ++i) {
		_q_target[i] = angle;
	}

	_command_mode = mode;
	_torque_requested = true;
	return true;
}

void DxlArm::processRequests()
{
	const int command = _requested_command.load();

	if (command != Command::NONE) {
		_requested_command.store(Command::NONE);
		requestCommand(static_cast<Command>(command));
	}
}

void DxlArm::run()
{
	if (!openPort()) {
		return;
	}

	requestCommand(Command::CENTER);
	PX4_INFO("Dynamixel driver running on TELEM1");

	hrt_abstime now = hrt_absolute_time();
	hrt_abstime next_feedback = now;
	hrt_abstime next_status = now;

	while (!should_exit()) {
		now = hrt_absolute_time();

		if (_parameter_update_sub.updated()) {
			updateParams();
		}

		updateRc(now);

		if (now >= next_feedback) {
			const bool valid = readFeedback();
			publishFeedback(valid, hrt_absolute_time());
			next_feedback = hrt_absolute_time() + 1000000 / _param_feedback_rate.get();
		}

		processRequests();
		now = hrt_absolute_time();

		if (_random_trajectory.active()) {
			_random_trajectory.update(now, _q_target);

			if (!_random_trajectory.active()) {
				_command_mode = dxl::RcMode::UNKNOWN;
				PX4_INFO("Random trajectory complete; holding the final target");
			}
		}

		if (_torque_requested && ensureTorque() && now >= _next_command) {
			writeGoals(_q_target);
			const hrt_abstime period = 1000000 / _param_command_rate.get();

			if (_next_command == 0 || now >= _next_command + period) {
				_next_command = now + period;

			} else {
				_next_command += period;
			}
		}

		if (now >= next_status) {
			publishStatus(now);
			next_status = now + 100000;
		}

		px4_usleep(2000);
	}

	closePort();
}

const char *DxlArm::stateName() const
{
	if (_random_trajectory.active()) {
		return "random";
	}

	switch (_command_mode) {
	case dxl::RcMode::UNKNOWN: return "hold";
	case dxl::RcMode::UP: return "raise";
	case dxl::RcMode::CENTER: return "center";
	case dxl::RcMode::DOWN: return "down";
	}

	return "unknown";
}

int DxlArm::print_status()
{
	PX4_INFO("state: %s, feedback: %s, torque: %s", stateName(), _feedback_valid ? "valid" : "invalid",
		 _torque_ready ? "enabled" : "not initialized");
	PX4_INFO("trajectory sample: %lu", (unsigned long)_random_trajectory.sampleIndex());
	PX4_INFO("sequence: %lu, period: %lu us, read latency: %lu us", (unsigned long)_feedback_sequence,
		 (unsigned long)_feedback_period_us, (unsigned long)_read_latency_us);
	PX4_INFO("read errors: %lu, write errors: %lu, RC mode: %d", (unsigned long)_read_errors,
		 (unsigned long)_write_errors, static_cast<int>(_rc_mode));
	return PX4_OK;
}

extern "C" __EXPORT int dxl_arm_main(int argc, char *argv[])
{
	return ModuleBase::main(DxlArm::desc, argc, argv);
}
