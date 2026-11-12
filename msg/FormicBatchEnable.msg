# Turns the publication of formic_gyro_batch and formic_accel_batch on or off (formic_imu_batch).
# Sent by the Jetson. The state returns to FORMIC_BATCH_DEF on disarm and is not stored.

uint64 timestamp	# time since system start (microseconds) elapsed

bool enable		# true: publish the batches, false: publish nothing
