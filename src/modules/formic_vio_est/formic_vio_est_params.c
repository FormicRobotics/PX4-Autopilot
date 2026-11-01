/**
 * Formic VIO estimator input
 *
 * Where formic_vio_est takes the VIO pose from.
 *
 * @group Formic VIO Estimator
 * @value 0 formic_odom (the Jetson bridge)
 * @value 1 vehicle_visual_odometry (Gazebo, simulation only)
 * @reboot_required true
 */
PARAM_DEFINE_INT32(FORMIC_VIO_SRC, 0);

/**
 * Formic VIO estimator largest pose delay
 *
 * A pose older than this when it arrives is dropped. Also the shortest IMU history kept.
 *
 * @group Formic VIO Estimator
 * @unit ms
 * @min 50
 * @max 600
 */
PARAM_DEFINE_INT32(FORMIC_VIO_DLY, 400);

/**
 * Formic VIO estimator timeout
 *
 * The VIO counts as stale, and the instance as unhealthy, when the newest pose used is older than
 * this. Position mode then fails over to Altitude mode.
 *
 * @group Formic VIO Estimator
 * @unit s
 * @min 0.1
 * @max 2.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FORMIC_VIO_TOUT, 0.3f);

/**
 * Formic VIO estimator attitude time constant
 *
 * How fast the output is pulled toward each VIO attitude.
 *
 * @group Formic VIO Estimator
 * @unit s
 * @min 0.02
 * @max 2.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FORMIC_VIO_TAU_A, 0.1f);

/**
 * Formic VIO estimator velocity time constant
 *
 * How fast the output is pulled toward each VIO velocity.
 *
 * @group Formic VIO Estimator
 * @unit s
 * @min 0.02
 * @max 2.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FORMIC_VIO_TAU_V, 0.25f);

/**
 * Formic VIO estimator position time constant
 *
 * How fast the output is pulled toward each VIO position.
 *
 * @group Formic VIO Estimator
 * @unit s
 * @min 0.02
 * @max 2.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FORMIC_VIO_TAU_P, 0.25f);
