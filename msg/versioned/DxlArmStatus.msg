# Dynamixel arm motion and bus diagnostics.

uint32 MESSAGE_VERSION = 0

uint64 timestamp
uint8 state
uint8 STATE_HOLD = 0
uint8 STATE_CENTERING = 1
uint8 STATE_CENTER_HOLD = 2
uint8 STATE_RANDOM = 3
uint8 STATE_FAULT = 4

bool torque_initialized # Goal seeding and torque-enable writes completed; not a torque register readback
bool feedback_valid
bool armed
bool rc_center_active
bool rc_random_active

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
