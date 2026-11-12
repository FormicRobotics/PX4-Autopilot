uint64 timestamp		# time since system start (microseconds)

uint8 primary_instance

uint8 instances_available

uint32 instance_changed_count
uint64 last_instance_change

uint32 accel_device_id
uint32 baro_device_id
uint32 gyro_device_id
uint32 mag_device_id

float32[9] combined_test_ratio
float32[9] relative_test_ratio
bool[9] healthy

float32[4] accumulated_gyro_error
float32[4] accumulated_accel_error
bool gyro_fault_detected
bool accel_fault_detected

# Formic: the role policy and the frame anchors (published = Rz(anchor_yaw) * instance + anchor_shift)
bool formic_role_active		# the role policy picks the primary instead of the error scores
uint8 role_primary		# instance the role policy wants
float32[2] anchor_yaw		# anchor of instance 0 and 1: rotation about down (rad)
float32[6] anchor_shift		# anchor shift NED (m): instance 0 xyz, then instance 1 xyz
bool[2] anchor_valid
uint32 vio_segment		# VIO segment the instance 1 anchor belongs to
bool ramp_active		# tilt ramp after a requested handover is running
uint32 handover_count		# handovers between EKF2 and the VIO instance
bool ekf_vel_seed_pending	# waiting for EKF2 to take the VIO horizontal velocity before switching to it
