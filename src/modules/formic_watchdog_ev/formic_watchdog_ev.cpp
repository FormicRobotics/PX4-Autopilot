#include "formic_watchdog_ev.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;
using namespace math;

void FormicWatchdogEv::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	_ev_hpos_enabled = (_param_ekf2_ev_ctrl.get() & (1 << 0)) != 0;
	_ev_yaw_enabled = (_param_ekf2_ev_ctrl.get() & (1 << 3)) != 0;

}


FormicWatchdogEv::FormicWatchdogEv() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);

}

bool FormicWatchdogEv::init()
{
	ScheduleOnInterval(10_ms);
	parameters_update();

	int32_t sens_imu_mode = 1;
	param_get(param_find("SENS_IMU_MODE"), &sens_imu_mode);

	if (sens_imu_mode == 0) {
		_estimtor_odometry_sub = uORB::Subscription{ORB_ID(estimator_odometry)};
		PX4_INFO("multi-EKF mode: using estimator_odometry");

	} else {
		_estimtor_odometry_sub = uORB::Subscription{ORB_ID(vehicle_odometry)};
		PX4_INFO("single-EKF mode: using vehicle_odometry");
	}

	return true;

}

void FormicWatchdogEv::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	if (_param_formic_wdev_en.get() == 0) {
		return;
	}

	handle_pos_req_user_intention(); // check if the commander requested to be in a position mode (e.g. by moving the AUX switch or by requesting a position mode while EV was not healthy, which triggers the position request as a fallback)

	formic_vehicle_odometry_s formic_odometry{};

	if (_odometry_sub_formic.update(&formic_odometry)) {
		_last_ev_timestamp = hrt_absolute_time();
		vehicle_odometry_s &odometry = formic_odometry.odometry;
		copy_odometry_msg(odometry);
		parse_status(formic_odometry.status);
		
	}


	publish_msg();
	no_EvData(); // updates _ev_data_arrived every cycle
}

// -----------------------------------------------------------------------------
// Data handling helpers
// -----------------------------------------------------------------------------

void FormicWatchdogEv::copy_odometry_msg(vehicle_odometry_s &odometry)

{
	// Always re-evaluate EKF/EV convergence this cycle, but only bump reset_counter
	// at most once every 500 ms so we don't force an EKF reset-to-vision on every sample.

	if (resetcounter_req(odometry) && (_last_reset_time == 0 || hrt_elapsed_time(&_last_reset_time) >= 500_ms)) {
		++reset_counter;
		_last_reset_time = hrt_absolute_time();
	}

	odometry.reset_counter = reset_counter;
	odometry.timestamp_sample = hrt_absolute_time(); // not ok - vlad 

	if (_pos_requested) {
		_odometry_pub.publish(odometry);
	}
	else {
		_heading_alligned_with_ev = false;
		_pos_alligned_with_ev = false;
		_ekfs_conv = false;
	}

}


void FormicWatchdogEv::handle_pos_req_user_intention()
{
	if (_formic_pos_req.updated()) {
		formic_pos_req_s pos_req{};
		_formic_pos_req.copy(&pos_req);
		_pos_requested = pos_req.pos_req;
	}
}


float FormicWatchdogEv::get_yaw_from_quat(const vehicle_odometry_s &odometry)
{
	const matrix::Quatf quat(odometry.q);
	if (!quat.isAllFinite() || quat.length() < 0.9f) {
		return NAN;
	}
	return matrix::Eulerf(quat).psi();
}


bool FormicWatchdogEv::check_EV_aid_src_heading(float vio_yaw, float estimator_yaw)
{
	estimator_aid_source1d_s ev_yaw{};
	if (!_estimator_aid_src_heading_sub.copy(&ev_yaw)) {
		_heading_alligned_with_ev = false;
		return false;
	}
	if (!ev_yaw.fused || ev_yaw.innovation_rejected) {
		_heading_alligned_with_ev = false;
		return false;
	}

	const float d_yaw = matrix::wrap_pi(vio_yaw - estimator_yaw);
	const float dyaw_thr = _param_formic_wdev_dyaw.get();
	_heading_alligned_with_ev = fabsf(d_yaw) <= dyaw_thr;
	return true; // had fused EV yaw aid data this cycle
}


bool FormicWatchdogEv::check_EV_aid_src_pos(const float vio_pos[2], const float estimator_pos[2])
{
	estimator_aid_source2d_s ev_pos{};

	if (!_estimator_aid_src_pos_sub.copy(&ev_pos)) {
		_pos_alligned_with_ev = false;
		return false;
	}

	if (!ev_pos.fused || ev_pos.innovation_rejected) {
		_pos_alligned_with_ev = false;
		return false;
	}

	matrix::Vector2f d_pos;
	for (int i = 0; i < 2; i++) {
		d_pos(i) = fabsf(vio_pos[i] - estimator_pos[i]);
	}
	_pos_alligned_with_ev = d_pos.norm() <= _param_formic_wdev_dpos.get();
	return true; // had fused EV position aid data this cycle
}


bool FormicWatchdogEv::resetcounter_req(vehicle_odometry_s &odometry){
	vehicle_odometry_s esti_odom{};

	if (!_estimtor_odometry_sub.copy(&esti_odom)) {
		return false;
	}

	const float raw_yaw      = get_yaw_from_quat(odometry);  // EV (VIO) yaw
	const float estimtor_yaw = get_yaw_from_quat(esti_odom); // EKF yaw
	const bool yaw_data_valid = _ev_yaw_enabled ? check_EV_aid_src_heading(raw_yaw, estimtor_yaw) : (_heading_alligned_with_ev = true, true);
	const bool pos_data_valid = _ev_hpos_enabled  ? check_EV_aid_src_pos(odometry.position, esti_odom.position) : (_pos_alligned_with_ev = true, true);

	const bool converged_now = yaw_data_valid && pos_data_valid
			     && _heading_alligned_with_ev
			     && _pos_alligned_with_ev;

	// Latch: once converged, stay converged through transient misalignment blips.
	// Only a session stop (copy_odometry_msg) or an EV dropout (no_EvData) clears it.
	_ekfs_conv = _ekfs_conv || converged_now;

	return !_ekfs_conv;
}




void FormicWatchdogEv::publish_msg()
{
        formic_ev_flag_s _formic_ev_flag{};
        _formic_ev_flag.timestamp = hrt_absolute_time();
        _formic_ev_flag.ev_data_arrived = _data_arrived;
        _formic_ev_flag.ekfs_converged = _ekfs_conv;
        _formic_ev_flag.heading_ok = _heading_alligned_with_ev;
        _formic_ev_flag.pos_ok = _pos_alligned_with_ev;
        _formic_ev_flag.dstate = static_cast<uint8_t>(_dstate);

        _formic_ev_flag_pub.publish(_formic_ev_flag);
}


void FormicWatchdogEv::no_EvData()
{
	/* Determine if there has been a dropout in the EV (Extended Visual) data stream. */
	if ((_last_ev_timestamp == 0) ||

	    ((hrt_absolute_time() - _last_ev_timestamp) > (hrt_abstime)(_param_ekf2_noaid_tout.get() * 1.5))) {
		_data_arrived = false;
		_heading_alligned_with_ev = false;
		_pos_alligned_with_ev = false;
		_ekfs_conv = false; // convergence can't hold once the EV stream has dropped out
		_last_reset_time = 0; // clear the 3 s reset throttle timer
		reset_counter = 0; // reset the EV reset counter at dropout, so the next session starts from zero
		_dstate = Dstate::NONE; // EV dropout state

	}
	else {
		// Fresh EV data has arrived within the timeout window.
		_data_arrived = true;
	}
}


void FormicWatchdogEv::parse_status(const char status[16])
{
	// Collapse the 7 VIOStatus values into the 4 Dstate values published in formic_ev_flag.
	if (strncmp(status, "NOMINAL", 16) == 0 || strncmp(status, "VALIDATING", 16) == 0) {
		_dstate = Dstate::VisionON;

	} else if (strncmp(status, "DEGRADED", 16) == 0) {
		_dstate = Dstate::DEGRADED;

	} else {
		// INACTIVE, INITIALIZING, RESETTING (no usable pose yet), or unknown
		_dstate = Dstate::NONE;
	}
}


int FormicWatchdogEv::task_spawn(int argc, char *argv[])
{
	FormicWatchdogEv *instance = new FormicWatchdogEv();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		PX4_INFO("started");
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}





FormicWatchdogEv *FormicWatchdogEv::instantiate(int argc, char *argv[])
{
	return new FormicWatchdogEv();
}

int FormicWatchdogEv::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormicWatchdogEv::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal skeleton module ready for custom logic.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_watchdog_ev", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	return PX4_OK;
}

extern "C" __EXPORT int formic_watchdog_ev_main(int argc, char *argv[])
{
	return FormicWatchdogEv::main(argc, argv);
}
