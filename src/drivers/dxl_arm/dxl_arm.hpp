#pragma once

#include "dxl_motion.hpp"
#include "dxl_protocol.hpp"
#include "dxl_rc.hpp"

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/delta_arm_state.h>
#include <uORB/topics/dxl_arm_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_status.h>

class DxlArm : public ModuleBase, public ModuleParams
{
public:
	DxlArm();
	~DxlArm() override;

	static Descriptor desc;
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int run_trampoline(int argc, char *argv[]);

	void run() override;
	int print_status() override;

private:
	enum Command : int {
		NONE = 0,
		CENTER = 1,
		RANDOM = 2,
		HOLD = 3,
		RAISE = 4,
	};

	bool openPort();
	void closePort();
	bool validConfiguration() const;
	int servoId(uint8_t index) const;
	int zeroCount(uint8_t index) const;
	int direction(uint8_t index) const;
	void configureMotion();

	bool writeAll(const uint8_t *data, size_t length, uint32_t timeout_us);
	int exchange(const uint8_t *request, size_t request_length, dxl::StatusPacket replies[3],
		     uint32_t timeout_us);
	bool readFeedback();
	bool writeGoals(const float positions[3]);
	bool setTorque(bool enabled);
	bool ensureTorque();

	void publishFeedback(bool valid, hrt_abstime timestamp);
	void publishStatus(hrt_abstime timestamp);
	void updateVehicleStatus();
	void updateRc(hrt_abstime timestamp);
	void processRequests(hrt_abstime timestamp);
	bool requestCommand(Command command, hrt_abstime timestamp);
	const char *stateName() const;

	int _fd{-1};
	px4::atomic<int> _requested_command{Command::NONE};
	dxl::MotionController _motion;

	float _q_actual[3] {};
	int32_t _raw_positions[3] {};
	bool _feedback_valid{false};
	bool _feedback_seen{false};
	bool _torque_ready{false};
	bool _torque_requested{false};
	uint32_t _feedback_sequence{0};
	uint32_t _read_latency_us{0};
	uint32_t _feedback_period_us{0};
	uint32_t _read_errors{0};
	uint32_t _write_errors{0};
	uint8_t _dxl_errors[3] {};
	hrt_abstime _last_valid_feedback{0};

	bool _armed{false};
	bool _vehicle_status_seen{false};
	bool _auto_center_pending{true};
	input_rc_s _input_rc{};
	bool _input_rc_seen{false};
	bool _rc_fresh{false};
	dxl::RcSwitch _rc_switch;
	dxl::RcMode _rc_mode{dxl::RcMode::UNKNOWN};
	Command _pending_rc_command{Command::NONE};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Publication<delta_arm_state_s> _delta_arm_state_pub{ORB_ID(delta_arm_state)};
	uORB::Publication<dxl_arm_status_s> _dxl_arm_status_pub{ORB_ID(dxl_arm_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DXL_ARM_BAUD>) _param_baud,
		(ParamInt<px4::params::DXL_ID1>) _param_id1,
		(ParamInt<px4::params::DXL_ID2>) _param_id2,
		(ParamInt<px4::params::DXL_ID3>) _param_id3,
		(ParamInt<px4::params::DXL_ZERO1>) _param_zero1,
		(ParamInt<px4::params::DXL_ZERO2>) _param_zero2,
		(ParamInt<px4::params::DXL_ZERO3>) _param_zero3,
		(ParamInt<px4::params::DXL_SIGN1>) _param_sign1,
		(ParamInt<px4::params::DXL_SIGN2>) _param_sign2,
		(ParamInt<px4::params::DXL_SIGN3>) _param_sign3,
		(ParamFloat<px4::params::DXL_Q_MIN>) _param_q_min,
		(ParamFloat<px4::params::DXL_Q_MAX>) _param_q_max,
		(ParamFloat<px4::params::DXL_MAX_VEL>) _param_max_velocity,
		(ParamFloat<px4::params::DXL_MAX_ACC>) _param_max_acceleration,
		(ParamFloat<px4::params::DXL_RAND_AMP>) _param_random_amplitude,
		(ParamFloat<px4::params::DXL_CENTER_TOL>) _param_center_tolerance,
		(ParamInt<px4::params::DXL_RAND_SEED>) _param_random_seed,
		(ParamInt<px4::params::DXL_FB_HZ>) _param_feedback_rate,
		(ParamInt<px4::params::DXL_CMD_HZ>) _param_command_rate,
		(ParamInt<px4::params::DXL_RC_CHANNEL>) _param_rc_channel,
		(ParamInt<px4::params::DXL_RC_DB_MS>) _param_rc_debounce,
		(ParamBool<px4::params::DXL_ALLOW_ARMED>) _param_allow_armed
	)
};
