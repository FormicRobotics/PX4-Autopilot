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

/**
 * Formic position source
 *
 * Which estimator a mode that needs position flies on: the VIO (formic_vio_est) or GPS (EKF2).
 * In the two Auto settings the source arbiter of the EKF2 selector picks the first source that is
 * good and falls back to the other one. The two "only" settings never use the other source.
 * An RC source switch (FORMIC_SRC_RC) wins over this parameter.
 *
 * @group EKF2
 * @value 0 Auto, VIO first
 * @value 1 Auto, GPS first
 * @value 2 VIO only
 * @value 3 GPS only
 */
PARAM_DEFINE_INT32(FORMIC_EST_PREF, 0);

/**
 * Formic GPS good: satellites
 *
 * GPS counts as good with at least this many satellites used. Good GPS turns
 * off 2 satellites under it.
 *
 * @group EKF2
 * @min 4
 * @max 40
 */
PARAM_DEFINE_INT32(FORMIC_GPS_SAT, 10);

/**
 * Formic GPS good: HDOP
 *
 * GPS counts as good with an HDOP of at most this. Good GPS turns off 0.5 over it.
 *
 * @group EKF2
 * @min 0.5
 * @max 10.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(FORMIC_GPS_HDOP, 1.5f);

/**
 * Formic GPS good: horizontal accuracy
 *
 * GPS counts as good with a horizontal accuracy (eph) of at most this.
 *
 * @group EKF2
 * @unit m
 * @min 0.1
 * @max 20.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(FORMIC_GPS_EPH, 2.0f);

/**
 * Formic GPS good: time to turn on
 *
 * GPS turns good after every check has passed for this long.
 *
 * @group EKF2
 * @unit s
 * @min 0.0
 * @max 60.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(FORMIC_GPS_TGOOD, 5.0f);

/**
 * Formic GPS good: time to turn off
 *
 * Good GPS turns off after a check has failed for this long. A lost fix, jamming
 * and spoofing turn it off at once.
 *
 * @group EKF2
 * @unit s
 * @min 0.0
 * @max 10.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(FORMIC_GPS_TBAD, 1.0f);
