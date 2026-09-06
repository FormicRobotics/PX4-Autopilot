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
