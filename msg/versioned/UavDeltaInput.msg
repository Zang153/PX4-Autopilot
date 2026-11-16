# Exact input and output of the UAV Delta policy control step.

uint32 MESSAGE_VERSION = 0

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Sampling timestamp of the triggering angular velocity
bool active # True when the policy is allowed to publish actuator_motors
bool native_step # True when the GRU hidden state is committed at 100 Hz

uint8 OBSERVATION_DIM = 25
uint8 ACTION_DIM = 4
float32[25] observation # RAPTOR 22D observation followed by q_actual[3]
float32[4] action # Policy-space rotor action in RAPTOR order, clipped to [-1, 1]

# TOPICS uav_delta_input
