#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * formic_velo_vis maximum commanded velocity
 *
 * Clamp applied to formic_velo_vis_setpoint's vx/vy/vz before they are
 * copied into trajectory_setpoint for mc_pos_control's velocity controller.
 *
 * @min 0.1
 * @max 10.0
 * @unit m/s
 * @group Formic Velo Vis
 * @category Standard
 */
PARAM_DEFINE_FLOAT(FVV_VEL_MAX, 2.0f);
