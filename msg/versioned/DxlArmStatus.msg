# Dynamixel arm motion and bus diagnostics.

uint32 MESSAGE_VERSION = 0

uint64 timestamp
uint8 state
uint8 STATE_HOLD = 0
uint8 STATE_RAISE = 1
uint8 STATE_CENTER = 2
uint8 STATE_DOWN = 3
uint8 STATE_RANDOM = 4

bool torque_initialized # Torque-enable write completed; not a torque register readback
bool feedback_valid
bool trajectory_active
uint16 trajectory_sample # Current fixed random trajectory sample, 0 to 3000 inclusive
uint8 rc_mode
uint8 RC_MODE_UNKNOWN = 0
uint8 RC_MODE_UP = 1
uint8 RC_MODE_CENTER = 2
uint8 RC_MODE_DOWN = 3

uint32 feedback_sequence
uint32 feedback_period_us
uint32 read_latency_us
uint32 read_errors
uint32 write_errors
int32[3] raw_position
float32[3] q_actual # [rad]
float32[3] q_target # [rad]
uint8[3] dxl_error

# TOPICS dxl_arm_status
