#pragma once

// PX4 platform / module framework
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

// uORB
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/formic_velo_vis_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/formic_velo_vis_status.h>

// Drivers
#include <drivers/drv_hrt.h>

// NAVIGATION_STATE_FORMIC_VELO_VIS's trajectory-setpoint generator. Unlike
// FormicVisServo (which bypasses mc_pos_control and publishes
// vehicle_attitude_setpoint directly from a tilt+climb-rate command),
// this mode wants true closed-loop velocity control on all 3 axes, so it
// deliberately feeds mc_pos_control's own, already-tuned velocity
// controller instead of re-deriving one: formic_velo_vis_setpoint's
// vx/vy/vz (NED, same convention as TrajectorySetpoint) are copied straight
// into trajectory_setpoint with position/acceleration/jerk left NaN, same
// shape as NAVIGATION_STATE_OFFBOARD's offboard_control_mode.velocity path.
// See ModeUtil/control_mode.cpp's NAVIGATION_STATE_FORMIC_VELO_VIS case for
// the control_mode flags this relies on (flag_control_velocity_enabled,
// no flag_control_position_enabled).
//
// Yaw: mc_pos_control's PositionControl does not integrate yawspeed itself -
// a NaN trajectory_setpoint.yaw just holds the current heading every tick
// (see PositionControl.cpp). So, same as FormicVisServo's own yaw handling,
// psi_dot_cmd is integrated here into an absolute yaw setpoint (with the
// same anti-windup lead-clamp, kYawSpMaxLead) and published as both yaw and
// yawspeed (feedforward).
//
// Entry/runtime gating (only run with a live command and a valid local
// velocity estimate, fall back to ALTCTL if the stream goes stale)
// lives in commander - see HealthAndArmingChecks/checks/formicVeloVisCheck.cpp
// and failsafe/failsafe.cpp's checkModeFallback(). This module itself just
// stays idle and publishes nothing whenever nav_state != FORMIC_VELO_VIS.
class FormicVeloVis : public ModuleBase<FormicVeloVis>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FormicVeloVis();
	~FormicVeloVis() override;

	static int task_spawn(int argc, char *argv[]);
	static FormicVeloVis *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();
	void Run() override;

private:
	// What Run() did on its last tick. Every change is logged (PX4_INFO /
	// PX4_WARN, so it lands in the ulog's logged messages), to tell apart
	// the ways this module can end up publishing nothing while nav_state is
	// FORMIC_VELO_VIS.
	enum class TickResult : uint8_t {
		Inactive,      ///< nav_state != FORMIC_VELO_VIS
		EntryTick,     ///< first active tick, state reset, nothing published
		NoSetpoint,    ///< formic_velo_vis_setpoint copy() failed
		ZeroTimestamp, ///< formic_velo_vis_setpoint copied but timestamp == 0
		Published,     ///< trajectory_setpoint published
	};
	static const char *tick_result_str(TickResult result);
	void set_tick_result(TickResult result, const formic_velo_vis_setpoint_s *setpoint = nullptr);

	void run_once();
	void parameters_update(bool force = false);
	void reset_on_entry(float current_yaw);
	void publish_trajectory_setpoint(const formic_velo_vis_setpoint_s &setpoint, float dt, float current_yaw);
	// FVV_VEL_MAX out to the off-board controller, ~1 Hz in every nav_state
	// (it needs it before the mode is entered).
	void publish_status();

	// --- Subscriptions ---
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _formic_velo_vis_setpoint_sub{ORB_ID(formic_velo_vis_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// --- Publications ---
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<formic_velo_vis_status_s> _formic_velo_vis_status_pub{ORB_ID(formic_velo_vis_status)};
	hrt_abstime _last_status_pub{0};

	// --- State ---
	bool _was_active{false}; ///< nav_state == FORMIC_VELO_VIS on the previous tick
	hrt_abstime _last_run{0};
	float _yaw_setpoint{0.f}; ///< heading-hold setpoint, integrated from psi_dot_cmd

	TickResult _tick_result{TickResult::Inactive};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _active_perf{perf_alloc(PC_COUNT, MODULE_NAME": active ticks")};
	perf_counter_t _no_setpoint_perf{perf_alloc(PC_COUNT, MODULE_NAME": no setpoint")};
	perf_counter_t _zero_ts_perf{perf_alloc(PC_COUNT, MODULE_NAME": zero timestamp")};
	perf_counter_t _published_perf{perf_alloc(PC_COUNT, MODULE_NAME": published")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FVV_VEL_MAX>) _param_formic_velo_vis_vel_max
	)
};
