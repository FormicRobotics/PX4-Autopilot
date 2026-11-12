# Status of formic_vio_est, estimator instance 1.

uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample			# sample time of the newest accepted VIO pose, PX4 clock (microseconds)

bool initialised			# the output has been started from a VIO pose
bool healthy				# newest pose younger than FORMIC_VIO_TOUT, VIO status NOMINAL or DEGRADED, clock locked
bool pos_offerable			# healthy and VIO status NOMINAL: Position mode may be entered on this instance
bool gap_exceeded			# no pose accepted for FORMIC_VIO_GAP_MAX: the instance must not fly
bool vel_trusted			# the velocity may seed EKF2 at a handover: not gap_exceeded and VIO status not DIVERGED
bool clock_locked			# the newest pose carried a usable sender age

uint8 vio_status			# status of the newest pose received
uint8 VIO_STATUS_INACTIVE = 0
uint8 VIO_STATUS_INITIALIZING = 1
uint8 VIO_STATUS_VALIDATING = 2
uint8 VIO_STATUS_NOMINAL = 3
uint8 VIO_STATUS_DEGRADED = 4
uint8 VIO_STATUS_DIVERGED = 5
uint8 VIO_STATUS_RESETTING = 6
uint8 VIO_STATUS_UNKNOWN = 7

uint64 epoch				# VIO epoch (restart counter) of the newest accepted pose
uint32 segment				# VIO segment: changes when the output frame is not continuous (after a long gap)
uint32 epoch_changes			# VIO restarts anchored onto the output within the segment
float32 epoch_anchor_yaw		# map from the current epoch's frame to the segment frame: rotation about down (rad); 0 while epoch_changes is 0
float32[3] epoch_anchor_shift		# and shift, applied after the rotation: p_seg = Rz(yaw) * p_epoch + shift (m, NED); 0 while epoch_changes is 0

uint32 sample_age_us			# age of the newest accepted pose now
uint32 latency_us			# age of the newest pose when it arrived
uint32[8] latency_hist			# arrival latency of accepted poses, 50 ms bins, the last bin 350 ms and more

float32 err_att				# tracking error at the newest accepted pose, before correction: attitude (rad)
float32 err_vel				# velocity (m/s)
float32 err_pos				# position (m)
float32 tilt_diff_ekf			# tilt between this instance and EKF2 (instance 0) (rad)

uint32 n_accepted			# poses used
uint32 n_rej_status			# poses dropped: status not NOMINAL or DEGRADED
uint32 n_rej_stamp			# poses dropped: the sender stamps are not an age (clock not locked)
uint32 n_rej_old			# poses dropped: older than FORMIC_VIO_DLY_MAX or the IMU history
uint32 n_rej_future			# poses dropped: newer than the newest IMU sample
uint32 n_rej_invalid			# poses dropped: frame not supported or values not finite
