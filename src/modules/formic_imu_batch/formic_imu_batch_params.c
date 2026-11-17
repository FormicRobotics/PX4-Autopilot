#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * Formic IMU batch: publish by default.
 *
 * The state of formic_gyro_batch and formic_accel_batch at boot and after every disarm.
 * A formic_batch_enable message (/fmu/in/formic_batch_enable) turns them on or off until the
 * next disarm. A change of this parameter applies at once.
 *
 * @group Formic IMU Batch
 * @category Standard
 * @value 0 Off
 * @value 1 Publishing
 */
PARAM_DEFINE_INT32(FORMIC_BATCH_DEF, 1);

/**
 * Formic IMU: the IMU the vehicle flies on.
 *
 * On a board with two IMUs (aerium_radian_h7u), the IMU its rc.board_sensors starts first. That
 * IMU becomes vehicle_imu instance 0: EKF2 (estimator instance 0), formic_vio_est (instance 1),
 * the rate controller and formic_gyro_batch / formic_accel_batch all use it. If it does not start,
 * the other IMU takes its place. Boards with one IMU ignore this parameter.
 *
 * The ICM42688P samples at 8 kHz; formic_imu_batch averages it by 5 to about 1.6 kHz, as the BMI270.
 * The rate controller runs on its 8 kHz samples: check the gyro filters (IMU_GYRO_CUTOFF, notches)
 * before flying on it.
 *
 * @group Formic IMU Batch
 * @category Standard
 * @value 0 BMI270
 * @value 1 ICM42688P
 * @reboot_required true
 */
PARAM_DEFINE_INT32(FORMIC_IMU_SEL, 0);
