# A batch of raw samples of one sensor of the IMU EKF2 flies on (FORMIC_IMU_SEL), for the Jetson.
# Published by formic_imu_batch. Every sample of the driver's FIFO up to 1.6 kHz; a faster sensor is
# averaged down to about 1.6 kHz (8 kHz ICM42688P: each sample the mean of 5 consecutive FIFO samples,
# at their mean time). Rotated into the FRD board frame by the driver, as sensor_*_fifo.

uint64 timestamp		# time of publication (microseconds)
uint64 timestamp_sample		# sample time of the first sample (microseconds)

uint32 device_id		# the gyro (formic_gyro_batch) or accel (formic_accel_batch) the samples came from

float32 scale			# value = raw * scale: rad/s (gyro) or m/s^2 (accel)

float32 temperature		# sensor temperature (degrees Celsius) at publication, from the driver's sensor_gyro / sensor_accel of device_id; NaN when unknown

uint8 samples			# number of valid samples
uint8 MAX_SAMPLES = 32

uint32 samples_lost		# batch samples of this sensor missing on the FMU since the module started, found by sample time (cumulative); a gap beyond its step since the previous message was lost on the link

uint16[32] dt			# sample time of each sample after timestamp_sample (microseconds); a gap of more than one sample interval means samples were lost

int16[32] x			# FRD board frame X-axis
int16[32] y			# FRD board frame Y-axis
int16[32] z			# FRD board frame Z-axis

uint8 ORB_QUEUE_LENGTH = 4

# TOPICS formic_gyro_batch formic_accel_batch
