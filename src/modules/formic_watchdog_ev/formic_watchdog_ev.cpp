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
		}


	no_EvData(); // updates _ev_data_arrived every cycle
}

// -----------------------------------------------------------------------------
// Data handling helpers
// -----------------------------------------------------------------------------

void FormicWatchdogEv::copy_odometry_msg(vehicle_odometry_s &odometry)

{
	odometry.reset_counter = resetcounter_req(odometry) ? ++reset_counter : reset_counter;
	_odometry_pub.publish(odometry);
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
	const bool yaw_data_valid = check_EV_aid_src_heading(raw_yaw, estimtor_yaw);
	const bool pos_data_valid = _ev_hpos_enabled  ? check_EV_aid_src_pos(odometry.position, esti_odom.position) : (_formic_ev_flag.pos_ok = true, true);
	ekfs_conv = yaw_data_valid && pos_data_valid
			     && _heading_alligned_with_ev
			     && _pos_alligned_with_ev;

	return ekfs_conv;









} 


// void FormicWatchdogEv::resetcounter(vehicle_odometry_s &odometry)
// {
// 	vehicle_odometry_s esti_odom{};

// 	if (!_estimtor_odometry_sub.copy(&esti_odom)) {
// 		return;
// 	}

// 	const float raw_yaw      = get_yaw_from_quat(odometry);  // EV (VIO) yaw
// 	const float estimtor_yaw = get_yaw_from_quat(esti_odom); // EKF yaw


// 	if (!_formic_ev_flag.ev_data_arrived) {
// 		reset_counter = 0;
// 		_last_reset_time = 0;
// 		at_reset_counter = true;
// 		return;
// 	}

// 	const bool yaw_data_valid = check_EV_aid_src_heading(raw_yaw, estimtor_yaw);
// 	const bool pos_data_valid = _ev_hpos_enabled  ? handle_pos_reset(odometry.position, esti_odom.position) : (_formic_state.pos_alligned_with_ev = true, true);
	
// 	if (!at_reset_counter) {
// 		return;
// 	}

// 	const bool aligned = yaw_data_valid && pos_data_valid
// 			     && _heading_alligned_with_ev
// 			     && _formic_state.pos_alligned_with_ev;

// 	if (aligned) {
// 		at_reset_counter = false;
// 		return;
// 	}

// 	if (_last_reset_time != 0 && hrt_elapsed_time(&_last_reset_time) < 2_s) {
// 		return;
// 	}

// 	reset_counter++;
// 	_last_reset_time = hrt_absolute_time();
// }





void FormicWatchdogEv::no_EvData()
{
	/* Determine if there has been a dropout in the EV (Extended Visual) data stream. */
	if ((_last_ev_timestamp == 0) ||
	    ((hrt_absolute_time() - _last_ev_timestamp) > 700_ms)) {
		_formic_ev_flag.ev_data_arrived = false;		
		_formic_state.error_find = false; // dropout = end of session: clear latched error so the next session may use EV
		_heading_alligned_with_ev = false;
		_formic_state.pos_alligned_with_ev = false;
		_last_reset_time = 0; // clear the 3 s reset throttle timer
		at_reset_counter = true; // re-arm the reset phase: next session resets until the heading aligns, then checks for errors
		reset_counter = 0; // reset the EV reset counter at dropout, so the next session starts from zero

	}
	else {
		// Fresh EV data has arrived within the timeout window.
		_formic_ev_flag.ev_data_arrived = true;
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
