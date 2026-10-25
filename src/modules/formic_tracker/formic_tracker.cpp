#include "formic_tracker.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

namespace
{
// kThrustFloor mirrors offboard_control.py's own constant. kVzIntegralLimit
// was widened from that file's 0.3 to 0.5: live SITL testing (regress9,
// see formic_tracker.hpp's header comment) showed the vz loop needed more
// integral authority than that file ever required, because this fallback
// hover_thrust (FMTRK_HOVER_FB) is only an estimate - 0.3 left no room to
// also cancel a baseline error on top of actually tracking the commanded
// climb rate.
constexpr float kVzIntegralLimit = 0.5f;  // clamp on the thrust integral term (normalized thrust)
constexpr float kThrustFloor = -0.02f;    // thrust_z never let past this toward zero/positive
}

FormicTracker::FormicTracker() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
}

FormicTracker::~FormicTracker()
{
	perf_free(_cycle_perf);
	perf_free(_active_perf);
	perf_free(_no_setpoint_perf);
	perf_free(_zero_ts_perf);
	perf_free(_published_perf);
}

bool FormicTracker::init()
{
	ScheduleOnInterval(10_ms);
	parameters_update();
	return true;
}

void FormicTracker::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void FormicTracker::reset_on_entry(float current_yaw)
{
	_yaw_setpoint = current_yaw;
	_vz_integral = 0.f;
	_vz_prev_error = 0.f;
	_vz_pid_initialized = false;
}

const char *FormicTracker::tick_result_str(TickResult result)
{
	switch (result) {
	case TickResult::Inactive: return "inactive";

	case TickResult::EntryTick: return "entry tick";

	case TickResult::NoSetpoint: return "no formic_tracker_setpoint";

	case TickResult::ZeroTimestamp: return "formic_tracker_setpoint timestamp 0";

	case TickResult::Published: return "publishing";
	}

	return "?";
}

void FormicTracker::set_tick_result(TickResult result, const formic_tracker_setpoint_s *setpoint)
{
	if (result == _tick_result) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (result == TickResult::NoSetpoint || result == TickResult::ZeroTimestamp) {
		PX4_WARN("%s -> %s (now %" PRIu64 ")", tick_result_str(_tick_result), tick_result_str(result), now);

	} else if (setpoint != nullptr) {
		PX4_INFO("%s -> %s (now %" PRIu64 ", sp age %" PRId64 " us)", tick_result_str(_tick_result), tick_result_str(result),
			 now, static_cast<int64_t>(now - setpoint->timestamp));

	} else {
		PX4_INFO("%s -> %s (now %" PRIu64 ")", tick_result_str(_tick_result), tick_result_str(result), now);
	}

	_tick_result = result;
}

void FormicTracker::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	run_once();
	perf_end(_cycle_perf);
}

void FormicTracker::run_once()
{
	parameters_update();

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);
	const bool active = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_FORMIC_TRACKER);

	if (!active) {
		_was_active = false;
		_last_run = 0;
		set_tick_result(TickResult::Inactive);
		return;
	}

	perf_count(_active_perf);

	vehicle_attitude_s vehicle_attitude{};
	_vehicle_attitude_sub.copy(&vehicle_attitude);

	if (!_was_active) {
		const matrix::Quatf q(vehicle_attitude.q);
		const float current_yaw = q.isAllFinite() ? matrix::Eulerf(q).psi() : 0.f;
		reset_on_entry(current_yaw);
		_was_active = true;
		_last_run = hrt_absolute_time();
		set_tick_result(TickResult::EntryTick);
		// First tick after entry: nothing to integrate against yet, same as
		// offboard_control.py's "ATT" handler resetting state and letting the
		// next timer tick do the first real publish.
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.0002f, 0.1f);
	_last_run = now;

	formic_tracker_setpoint_s setpoint{};

	// Nothing received yet - commander's own entry/runtime gate
	// (formicTrackerCheck) is what keeps the vehicle out of this nav_state
	// without a live stream; this is just defense in depth against a race
	// right at mode entry. Publish nothing this tick.
	if (!_formic_tracker_setpoint_sub.copy(&setpoint)) {
		perf_count(_no_setpoint_perf);
		set_tick_result(TickResult::NoSetpoint);
		return;
	}

	if (setpoint.timestamp == 0) {
		perf_count(_zero_ts_perf);
		set_tick_result(TickResult::ZeroTimestamp);
		return;
	}

	vehicle_local_position_s vehicle_local_position{};
	_vehicle_local_position_sub.copy(&vehicle_local_position);

	hover_thrust_estimate_s hover_thrust_estimate{};

	if (_hover_thrust_estimate_sub.copy(&hover_thrust_estimate) && hover_thrust_estimate.valid) {
		_hover_thrust = math::constrain(hover_thrust_estimate.hover_thrust, .05f, .9f);

	} else {
		_hover_thrust = _param_formic_trk_hover_fb.get();
	}

	publish_attitude_setpoint(setpoint, dt, vehicle_local_position.vz);
	perf_count(_published_perf);
	set_tick_result(TickResult::Published, &setpoint);
}

void FormicTracker::publish_attitude_setpoint(const formic_tracker_setpoint_s &setpoint, float dt, float vz_actual)
{
	const float tilt_max = _param_formic_trk_tilt_max.get();
	const float vz_max = _param_formic_trk_vz_max.get();

	// Roll/pitch/yaw -> q_d: exact port of offboard_control.py's
	// publish_attitude_setpoint() (roll/pitch commanded open-loop, yaw
	// integrated from the commanded yaw rate).
	_yaw_setpoint += setpoint.psi_dot_cmd * dt;

	const float roll = math::constrain(setpoint.phi_cmd, -tilt_max, tilt_max);
	const float pitch = math::constrain(setpoint.theta_cmd, -tilt_max, tilt_max);
	const float yaw = _yaw_setpoint;

	const float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
	const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
	const float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);

	const float q_d[4] = {
		cr * cp * cy + sr * sp * sy,
		sr * cp * cy - cr * sp * sy,
		cr * sp * cy + sr * cp * sy,
		cr * cp * sy - sr * sp * cy,
	};

	// vz -> thrust_z: exact port of the same function's climb-rate PID.
	// cmd is +up; PX4 local-position vz is +down.
	const float vz_cmd_ned = -math::constrain(setpoint.v_cmd, -vz_max, vz_max);
	const float vz_error = vz_cmd_ned - vz_actual;

	float derivative = 0.f;

	if (dt > 0.f) {
		_vz_integral = math::constrain(_vz_integral + vz_error * dt, -kVzIntegralLimit, kVzIntegralLimit);

		if (_vz_pid_initialized) {
			derivative = (vz_error - _vz_prev_error) / dt;
		}
	}

	_vz_prev_error = vz_error;
	_vz_pid_initialized = true;

	const float output_limit = _hover_thrust - kThrustFloor;
	const float thrust_correction = math::constrain(
			_param_formic_trk_vz_p.get() * vz_error
			+ _param_formic_trk_vz_i.get() * _vz_integral
			+ _param_formic_trk_vz_d.get() * derivative,
			-output_limit, output_limit);

	float thrust_z = -_hover_thrust + thrust_correction;
	thrust_z = math::constrain(thrust_z, -1.f, kThrustFloor);

	vehicle_attitude_setpoint_s attitude_setpoint{};
	attitude_setpoint.timestamp = hrt_absolute_time();
	attitude_setpoint.q_d[0] = q_d[0];
	attitude_setpoint.q_d[1] = q_d[1];
	attitude_setpoint.q_d[2] = q_d[2];
	attitude_setpoint.q_d[3] = q_d[3];
	attitude_setpoint.thrust_body[0] = 0.f;
	attitude_setpoint.thrust_body[1] = 0.f;
	attitude_setpoint.thrust_body[2] = thrust_z;
	attitude_setpoint.yaw_sp_move_rate = setpoint.psi_dot_cmd;
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

int FormicTracker::task_spawn(int argc, char *argv[])
{
	FormicTracker *instance = new FormicTracker();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

FormicTracker *FormicTracker::instantiate(int argc, char *argv[])
{
	return new FormicTracker();
}

int FormicTracker::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormicTracker::print_status()
{
	PX4_INFO("last tick: %s, setpoint sub valid: %d", tick_result_str(_tick_result), _formic_tracker_setpoint_sub.valid());
	perf_print_counter(_cycle_perf);
	perf_print_counter(_active_perf);
	perf_print_counter(_no_setpoint_perf);
	perf_print_counter(_zero_ts_perf);
	perf_print_counter(_published_perf);
	return 0;
}

int FormicTracker::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Attitude+climb-rate setpoint generator for NAVIGATION_STATE_FORMIC_TRACKER.
Drives vehicle_attitude_setpoint directly from formic_tracker_setpoint
(roll/pitch/climb-rate/yaw-rate) while that nav_state is active; idle and
publishes nothing otherwise. See formic_tracker.hpp for the full design note.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_tracker", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	return PX4_OK;
}

extern "C" __EXPORT int formic_tracker_main(int argc, char *argv[])
{
	return FormicTracker::main(argc, argv);
}
