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
