#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * formic_tracker maximum commanded tilt
 *
 * Clamp applied to formic_tracker_setpoint's phi_cmd/theta_cmd before they
 * are used as the roll/pitch attitude setpoint.
 *
 * @min 0.0
 * @max 0.78
 * @unit rad
 * @group Formic Tracker
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FMTRK_TILT_MAX, 0.35f);

/**
 * formic_tracker maximum commanded climb rate
 *
 * Clamp applied to formic_tracker_setpoint's v_cmd.
 *
 * @min 0.1
 * @max 10.0
 * @unit m/s
 * @group Formic Tracker
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FMTRK_VZ_MAX, 2.0f);

/**
 * formic_tracker climb-rate PID: proportional gain
 *
 * Raised from 0.2 after live SITL testing (regress9) showed the original
 * value left the vehicle unable to build enough net lift to leave the
 * ground before PX4's landed-disarm timeout - see formic_tracker.hpp's
 * header comment.
 *
 * @min 0.0
 * @max 2.0
 * @group Formic Tracker
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FMTRK_VZ_P, 0.9f);

/**
 * formic_tracker climb-rate PID: integral gain
 *
 * Raised from 0.05 alongside FMTRK_VZ_P - see that parameter's comment.
 *
 * @min 0.0
 * @max 1.0
 * @group Formic Tracker
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FMTRK_VZ_I, 0.2f);

/**
 * formic_tracker climb-rate PID: derivative gain
 *
 * Raised from 0.02 for damping against the larger P/I terms above.
 *
 * @min 0.0
 * @max 1.0
 * @group Formic Tracker
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FMTRK_VZ_D, 0.05f);

/**
 * formic_tracker climb-rate PID: hover-thrust fallback
 *
 * Baseline normalized thrust used by the vz loop whenever
 * hover_thrust_estimate isn't valid yet (always true right at liftoff,
 * before the estimator has converged). Deliberately not MPC_THR_HOVER's
 * generic default: this specific camera-carrying x500_vision SITL airframe
 * was already measured at ~0.69-0.71 hover thrust (see
 * offboard_control.py's own calibration comment) - see formic_tracker.hpp's
 * header comment for how this was confirmed live (regress9).
 *
 * @min 0.1
 * @max 0.9
 * @group Formic Tracker
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FMTRK_HOVER_FB, 0.70f);
