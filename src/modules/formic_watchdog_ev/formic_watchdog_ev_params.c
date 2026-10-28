#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * Enable the Formic Watchdog EV module.
 *
 * @min 0
 * @max 1
 * @group Formic Watchdog EV
 * @category Standard
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(FORMIC_WDEV_EN, 0);


/**
 * EV heading reset threshold.
 *
 * Minimum heading difference (radians) between the local estimate and the
 * EV yaw observation required to trigger a heading reset counter increment.
 * When the difference is smaller than this value the heading is considered
 * aligned and no reset is counted for this session. The reset is still
 * throttled to at most one increment every 2 seconds.
 *
 * @min 0.0
 * @max 3.14
 * @unit rad
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FORMIC_WDEV_DYAW, 0.5f);

/**
 * EV position reset threshold.
 *
 * Minimum horizontal distance (meters) between the local position estimate and
 * the EV position observation required to keep the session in the reset phase.
 * When the distance is smaller than this value the position is considered
 * aligned with the EV. The reset phase ends only once BOTH the heading
 * (FORMIC_WDEV_DYAW) and the position are aligned. Reset increments are still
 * throttled to at most one every 2 seconds.
 *
 * @min 0.0
 * @max 50.0
 * @unit m
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FORMIC_WDEV_DPOS, 1.0f);

/**
 * Largest age of a pose that is fused.
 *
 * The age of a pose is the time from the sample of the pose to the moment its sender
 * published it, the difference of the two stamps in formic_odom. The sample time that
 * EKF2 gets is the publish time minus the age, on the PX4 clock. A pose older than this
 * is dropped, counted ("formic_watchdog_ev status") and does not count as EV data.
 * 0 turns the sample time off: every pose gets the time it was polled, as before.
 * Keep it at most EKF2_DELAY_MAX. EKF2_EV_DELAY is then the delay that is left over, near 0.
 *
 * @min 0
 * @max 1000
 * @unit ms
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_INT32(FORMIC_WDEV_AMAX, 200);

/**
 * Largest age that is taken as an age.
 *
 * A sender that does not stamp the sample on the same clock as the publish time, like
 * the control repo's openvins2px4 (VIO clock), gives a negative or huge "age". An age
 * above this, or below 0, is not an age: the pose gets the time it was polled.
 *
 * @min 0
 * @max 60000
 * @unit ms
 * @group Formic Watchdog EV
 * @category Standard
 */
PARAM_DEFINE_INT32(FORMIC_WDEV_AVLD, 5000);
