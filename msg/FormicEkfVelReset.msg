# FORMIC_EST_MODE 1: the EKF2 selector asks EKF2 to set its horizontal velocity at a handover from the
# VIO instance back to EKF2. The vertical velocity stays EKF2's.

uint64 timestamp		# time since system start (microseconds)

uint8 instance			# EKF2 instance to reset
float32[2] vel_ne		# horizontal velocity in the frame of that EKF2 instance, north and east (m/s)
float32 vel_var			# its variance (m^2/s^2)
uint8 sequence			# changes with every request
