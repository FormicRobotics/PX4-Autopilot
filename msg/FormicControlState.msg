# The state the controllers use: the vehicle_local_position and vehicle_attitude the EKF2 selector
# has just published, and the estimator instance they came from.

uint64 timestamp			# time of publication (microseconds)
uint64 timestamp_sample			# sample time of the position part, as in vehicle_local_position (microseconds)
uint64 attitude_timestamp_sample	# sample time of the attitude part, as in vehicle_attitude (microseconds)

uint8 instance				# estimator instance the controllers are fed from
uint8 INSTANCE_EKF2 = 0
uint8 INSTANCE_VIO = 1
uint32 handover_count			# handovers between the two instances so far
bool tilt_ramp_active			# the tilt ramp after a handover is running

float32[3] position			# local frame north-east-down (m), as in vehicle_local_position
float32[3] velocity			# local frame north-east-down (m/s)
float32 heading				# (rad)
float32[4] q				# rotation from the body frame (forward-right-down) to the local frame, (w, x, y, z), as in vehicle_attitude

bool xy_valid
bool z_valid
bool v_xy_valid
bool v_z_valid
float32 eph				# standard deviation of the horizontal position error (m)
float32 epv				# standard deviation of the vertical position error (m)
float32 evh				# standard deviation of the horizontal velocity error (m/s)
float32 evv				# standard deviation of the vertical velocity error (m/s)

uint8 xy_reset_counter
uint8 z_reset_counter
uint8 vxy_reset_counter
uint8 vz_reset_counter
uint8 heading_reset_counter
uint8 quat_reset_counter
