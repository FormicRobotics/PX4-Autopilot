#include "formic_velo_vis.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace time_literals;

namespace
{
// Anti-windup on the integrated yaw setpoint: it may lead the measured yaw
// by at most this much. Same rationale and value as FormicVisServo's own
// kYawSpMaxLead (see that module's header comment) - without it,
// psi_dot_cmd integrated while the vehicle could not yet turn (still on the
// ground right after entry) would pile up and the attitude loop would snap
// to it at liftoff.
constexpr float kYawSpMaxLead = 0.3f;     // rad
}

FormicVeloVis::FormicVeloVis() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
}

FormicVeloVis::~FormicVeloVis()
{
	perf_free(_cycle_perf);
	perf_free(_active_perf);
	perf_free(_no_setpoint_perf);
	perf_free(_zero_ts_perf);
	perf_free(_published_perf);
}

bool FormicVeloVis::init()
{
	ScheduleOnInterval(10_ms);
	parameters_update();
	return true;
}

void FormicVeloVis::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void FormicVeloVis::reset_on_entry(float current_yaw)
{
	_yaw_setpoint = current_yaw;
}

const char *FormicVeloVis::tick_result_str(TickResult result)
{
	switch (result) {
	case TickResult::Inactive: return "inactive";

	case TickResult::EntryTick: return "entry tick";

	case TickResult::NoSetpoint: return "no formic_velo_vis_setpoint";

	case TickResult::ZeroTimestamp: return "formic_velo_vis_setpoint timestamp 0";

	case TickResult::Published: return "publishing";
	}

	return "?";
}

void FormicVeloVis::set_tick_result(TickResult result, const formic_velo_vis_setpoint_s *setpoint)
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

void FormicVeloVis::Run()
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

void FormicVeloVis::publish_status()
{
	const hrt_abstime now = hrt_absolute_time();

	if (now - _last_status_pub < 1_s) {
		return;
	}

	_last_status_pub = now;
	formic_velo_vis_status_s status{};
	status.timestamp = now;
	status.vel_max = _param_formic_velo_vis_vel_max.get();
	_formic_velo_vis_status_pub.publish(status);
}

void FormicVeloVis::run_once()
{
	parameters_update();
	publish_status();

	vehicle_status_s vehicle_status{};
	_vehicle_status_sub.copy(&vehicle_status);
	const bool active = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_FORMIC_VELO_VIS);

	if (!active) {
		_was_active = false;
		_last_run = 0;
		set_tick_result(TickResult::Inactive);
		return;
	}

	perf_count(_active_perf);

	vehicle_local_position_s vehicle_local_position{};
	_vehicle_local_position_sub.copy(&vehicle_local_position);

	if (!_was_active) {
		const float current_yaw = PX4_ISFINITE(vehicle_local_position.heading) ? vehicle_local_position.heading : 0.f;
		reset_on_entry(current_yaw);
		_was_active = true;
		_last_run = hrt_absolute_time();
		set_tick_result(TickResult::EntryTick);
		// First tick after entry: nothing to integrate against yet.
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain((now - _last_run) * 1e-6f, 0.0002f, 0.1f);
	_last_run = now;

	formic_velo_vis_setpoint_s setpoint{};

	// Nothing received yet - commander's own entry/runtime gate
	// (formicVeloVisCheck) is what keeps the vehicle out of this nav_state
	// without a live stream; this is just defense in depth against a race
	// right at mode entry. Publish nothing this tick.
	if (!_formic_velo_vis_setpoint_sub.copy(&setpoint)) {
		perf_count(_no_setpoint_perf);
		set_tick_result(TickResult::NoSetpoint);
		return;
	}

	if (setpoint.timestamp == 0) {
		perf_count(_zero_ts_perf);
		set_tick_result(TickResult::ZeroTimestamp);
		return;
	}

	const float current_yaw = PX4_ISFINITE(vehicle_local_position.heading) ? vehicle_local_position.heading :
				   _yaw_setpoint;
	publish_trajectory_setpoint(setpoint, dt, current_yaw);
	perf_count(_published_perf);
	set_tick_result(TickResult::Published, &setpoint);
}

void FormicVeloVis::publish_trajectory_setpoint(const formic_velo_vis_setpoint_s &setpoint, float dt,
		float current_yaw)
{
	const float vel_max = _param_formic_velo_vis_vel_max.get();

	// psi_dot_cmd -> yaw: PositionControl does not integrate yawspeed itself
	// (a NaN trajectory_setpoint.yaw just holds the current heading every
	// tick), so this mirrors FormicVisServo's own yaw integration + the same
	// anti-windup lead-clamp.
	_yaw_setpoint += setpoint.psi_dot_cmd * dt;
	const float yaw_lead = math::constrain(matrix::wrap_pi(_yaw_setpoint - current_yaw), -kYawSpMaxLead, kYawSpMaxLead);
	_yaw_setpoint = matrix::wrap_pi(current_yaw + yaw_lead);

	trajectory_setpoint_s trajectory_setpoint{};
	trajectory_setpoint.timestamp = hrt_absolute_time();
	trajectory_setpoint.position[0] = NAN;
	trajectory_setpoint.position[1] = NAN;
	trajectory_setpoint.position[2] = NAN;
	trajectory_setpoint.velocity[0] = math::constrain(setpoint.vx, -vel_max, vel_max);
	trajectory_setpoint.velocity[1] = math::constrain(setpoint.vy, -vel_max, vel_max);
	trajectory_setpoint.velocity[2] = math::constrain(setpoint.vz, -vel_max, vel_max);
	trajectory_setpoint.acceleration[0] = NAN;
	trajectory_setpoint.acceleration[1] = NAN;
	trajectory_setpoint.acceleration[2] = NAN;
	trajectory_setpoint.jerk[0] = NAN;
	trajectory_setpoint.jerk[1] = NAN;
	trajectory_setpoint.jerk[2] = NAN;
	trajectory_setpoint.yaw = _yaw_setpoint;
	trajectory_setpoint.yawspeed = setpoint.psi_dot_cmd;
	_trajectory_setpoint_pub.publish(trajectory_setpoint);
}

int FormicVeloVis::task_spawn(int argc, char *argv[])
{
	FormicVeloVis *instance = new FormicVeloVis();

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

FormicVeloVis *FormicVeloVis::instantiate(int argc, char *argv[])
{
	return new FormicVeloVis();
}

int FormicVeloVis::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormicVeloVis::print_status()
{
	PX4_INFO("last tick: %s, setpoint sub valid: %d", tick_result_str(_tick_result), _formic_velo_vis_setpoint_sub.valid());
	perf_print_counter(_cycle_perf);
	perf_print_counter(_active_perf);
	perf_print_counter(_no_setpoint_perf);
	perf_print_counter(_zero_ts_perf);
	perf_print_counter(_published_perf);
	return 0;
}

int FormicVeloVis::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Trajectory-setpoint generator for NAVIGATION_STATE_FORMIC_VELO_VIS. Copies
formic_velo_vis_setpoint (vx/vy/vz/yaw-rate) into trajectory_setpoint while
that nav_state is active, letting mc_pos_control's own velocity controller
do the closed-loop work on all 3 axes; idle and publishes nothing otherwise.
See formic_velo_vis.hpp for the full design note.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_velo_vis", "module");
	PRINT_MODULE_USAGE_COMMAND("start");
	return PX4_OK;
}

extern "C" __EXPORT int formic_velo_vis_main(int argc, char *argv[])
{
	return FormicVeloVis::main(argc, argv);
}
