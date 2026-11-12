/**
 * Formic VIO longest gap
 *
 * The longest time formic_vio_est carries the output on the IMU alone. After
 * it, the VIO instance is unhealthy and the EKF2 selector switches to EKF2 even if Position mode
 * is still active. A VIO restart within this gap keeps the flight frame.
 *
 * @group EKF2
 * @unit s
 * @min 0.2
 * @max 3.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FORMIC_VIO_GAP, 0.5f);

/**
 * Formic handover tilt time constant
 *
 * At a requested handover between EKF2 and the VIO instance, the difference in
 * roll and pitch is removed from the published attitude with this time constant. A handover
 * because the VIO failed steps the tilt.
 *
 * @group EKF2
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(FORMIC_HO_TAU, 0.5f);

/**
 * Formic handover EKF2 velocity seed
 *
 * At a handover from the VIO instance back to EKF2, EKF2's horizontal velocity
 * is set to the VIO instance's first. EKF2's vertical velocity is kept.
 *
 * @group EKF2
 * @boolean
 */
PARAM_DEFINE_INT32(FORMIC_HO_VSEED, 1);
