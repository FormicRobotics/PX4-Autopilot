#include "formicEstCheck.hpp"

FormicEstChecks::FormicEstChecks()
{
	_boot_mode = _param_formic_est_mode.get();

	// Looked up at run time: they do not exist when EKF2 is built without multi-EKF support.
	_param_sens_imu_mode = param_find("SENS_IMU_MODE");
	_param_ekf2_multi_imu = param_find("EKF2_MULTI_IMU");
	_param_ekf2_multi_mag = param_find("EKF2_MULTI_MAG");
	_param_formic_vio_src = param_find("FORMIC_VIO_SRC");
	_param_ekf2_ev_ctrl = param_find("EKF2_EV_CTRL");

	updateParams();
}

void FormicEstChecks::updateParams()
{
	HealthAndArmingCheckBase::updateParams();

	if (_param_sens_imu_mode != PARAM_INVALID) { param_get(_param_sens_imu_mode, &_sens_imu_mode); }

	if (_param_ekf2_multi_imu != PARAM_INVALID) { param_get(_param_ekf2_multi_imu, &_ekf2_multi_imu); }

	if (_param_ekf2_multi_mag != PARAM_INVALID) { param_get(_param_ekf2_multi_mag, &_ekf2_multi_mag); }

	if (_param_formic_vio_src != PARAM_INVALID) { param_get(_param_formic_vio_src, &_formic_vio_src); }

	if (_param_ekf2_ev_ctrl != PARAM_INVALID) { param_get(_param_ekf2_ev_ctrl, &_ekf2_ev_ctrl); }
}

void FormicEstChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (_param_formic_est_mode.get() != _boot_mode) {
		/* EVENT
		 * @description
		 * The modules for the new value are started at boot.
		 *
		 * <profile name="dev">
		 * Reboot to apply <param>FORMIC_EST_MODE</param>.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::local_position_estimate,
					    events::ID("check_formic_est_mode_changed"),
					    events::Log::Error, "Reboot to apply FORMIC_EST_MODE");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: reboot to apply FORMIC_EST_MODE %" PRId32,
					     _param_formic_est_mode.get());
		}

		return;
	}

	if (_boot_mode != 1) {
		return;
	}

	// EKF2 drops to one instance when EKF2_MULTI_MAG is 0 or 1. A value of 0 for EKF2_MULTI_IMU is
	// raised to 1 by EKF2 itself.
	const bool multi_ekf_built = (_param_sens_imu_mode != PARAM_INVALID) && (_param_ekf2_multi_imu != PARAM_INVALID);
	const bool one_ekf2_instance = multi_ekf_built && (_sens_imu_mode == 0) && (_ekf2_multi_imu <= 1)
				       && (_param_ekf2_multi_mag == PARAM_INVALID || _ekf2_multi_mag <= 1);

	if (!one_ekf2_instance) {
		/* EVENT
		 * @description
		 * FORMIC_EST_MODE 1 runs EKF2 as estimator instance 0 behind the selector and formic_vio_est
		 * as instance 1.
		 *
		 * <profile name="dev">
		 * Set <param>SENS_IMU_MODE</param> to 0, <param>EKF2_MULTI_IMU</param> to 1 and
		 * <param>EKF2_MULTI_MAG</param> to 0 or 1, then reboot.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::local_position_estimate,
					    events::ID("check_formic_est_mode_ekf2_multi"),
					    events::Log::Error, "FORMIC_EST_MODE 1 needs SENS_IMU_MODE 0 and EKF2_MULTI_IMU 1");

		if (reporter.mavlink_log_pub()) {
			if (!multi_ekf_built) {
				mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: FORMIC_EST_MODE 1 needs multi-EKF");

			} else {
				mavlink_log_critical(reporter.mavlink_log_pub(),
						     "Preflight Fail: FORMIC_EST_MODE 1: SENS_IMU_MODE %" PRId32 " (0), EKF2_MULTI_IMU %" PRId32
						     " (1), EKF2_MULTI_MAG %" PRId32 " (<=1)", _sens_imu_mode, _ekf2_multi_imu, _ekf2_multi_mag);
			}
		}
	}

	// In simulation the VIO comes from vehicle_visual_odometry, which EKF2 would fuse too
	if (_formic_vio_src == 1 && _ekf2_ev_ctrl != 0) {
		/* EVENT
		 * @description
		 * With FORMIC_VIO_SRC 1, formic_vio_est reads vehicle_visual_odometry. EKF2 must not fuse it.
		 *
		 * <profile name="dev">
		 * Set <param>EKF2_EV_CTRL</param> to 0.
		 * </profile>
		 */
		reporter.armingCheckFailure(NavModes::All, health_component_t::local_position_estimate,
					    events::ID("check_formic_est_mode_ev_ctrl"),
					    events::Log::Error, "FORMIC_VIO_SRC 1 needs EKF2_EV_CTRL 0");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: FORMIC_VIO_SRC 1 needs EKF2_EV_CTRL 0");
		}
	}
}
