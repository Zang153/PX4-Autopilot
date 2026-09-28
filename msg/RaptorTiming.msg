# Low-overhead timing events for RAPTOR closed-loop latency diagnosis.
# timestamp is the PX4 HRT time at the local event.
# timestamp_sample is copied unchanged from the source vehicle_angular_velocity
# sample in native mode or from the companion PolicyOutput state sample carried
# in actuator_motors in companion mode; it is the correlation key.
# actuator_timestamp is copied from actuator_motors.timestamp for diagnostics;
# for DDS-originated commands it can be in the companion computer clock domain
# and must not be subtracted from a PX4 HRT timestamp.
uint64 timestamp
uint64 timestamp_sample
uint64 actuator_timestamp
uint8 stage
uint8 ORB_QUEUE_LENGTH = 16
uint8 STAGE_NATIVE_COMMAND = 0
uint8 STAGE_INPUT_CONSUMED = 1
uint8 STAGE_OUTPUT_PUBLISHED = 2
