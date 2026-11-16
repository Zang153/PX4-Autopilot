# Actual joint angles of the three Delta arm motors.
# Joint order follows delta_motor_1_hinge, delta_motor_2_hinge,
# delta_motor_3_hinge in the training observation contract.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Sampling timestamp of the joint feedback
uint32 sequence # Monotonically increasing source sequence number
bool valid # True when all three joint angles are valid measurements
float32[3] q_actual # [rad] Actual joint angles, raw and unnormalized

# TOPICS delta_arm_state
