# Diagnostic state of the UAV Delta controller.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
bool active # True when UAV_DELTA is the active navigation mode
bool can_arm # True when all required controller inputs are available and fresh
bool delta_arm_state_stale
bool vehicle_angular_velocity_stale
bool vehicle_local_position_stale
bool vehicle_attitude_stale
bool trajectory_setpoint_stale

uint8 exit_reason
uint8 EXIT_REASON_NONE = 0
uint8 EXIT_REASON_NO_ANGULAR_VELOCITY_UPDATE = 1
uint8 EXIT_REASON_NOT_ALL_OBSERVATIONS_SET = 2
uint8 EXIT_REASON_ANGULAR_VELOCITY_STALE = 3
uint8 EXIT_REASON_LOCAL_POSITION_STALE = 4
uint8 EXIT_REASON_ATTITUDE_STALE = 5
uint8 EXIT_REASON_DELTA_ARM_STATE_STALE = 6

uint64 timestamp_last_delta_arm_state # [us] Last accepted Delta feedback receipt time
uint32 last_delta_sequence # Sequence number of the last accepted Delta feedback
uint32 native_step_count # Number of native GRU updates since the last activation reset
uint32 intermediate_step_count # Number of intermediate forwards since the last activation reset

# TOPICS uav_delta_status
