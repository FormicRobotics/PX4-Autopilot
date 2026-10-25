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
#include <uORB/topics/formic_tracker_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

// Drivers
#include <drivers/drv_hrt.h>

// NAVIGATION_STATE_FORMIC_TRACKER's attitude+thrust generator. This is the
// FC-side port of what offboard_control.py's "ATTITUDE" control mode used to
// do from off-board over DDS (see that file's publish_attitude_setpoint() and
// its PID class) - roll/pitch are commanded open-loop from formic_tracker_setpoint,
// only the vertical axis (vz) is closed-loop here, same as that Python node.
//
// Publishes vehicle_attitude_setpoint directly (mc_att_control already
// subscribes it unconditionally) and deliberately bypasses mc_pos_control:
// formic_tracker_setpoint already carries final tilt angles + climb rate, not
// a position/acceleration demand for mc_pos_control to re-invert. See
// ModeUtil/control_mode.cpp's NAVIGATION_STATE_FORMIC_TRACKER case for the
// control_mode flags this relies on.
//
// Entry/runtime gating (only run with a live command, fall back to
// FormicAltHold if the stream goes stale) lives in commander - see
// HealthAndArmingChecks/checks/formicTrackerCheck.cpp and
// failsafe/failsafe.cpp's checkModeFallback(). This module itself just stays
// idle and publishes nothing whenever nav_state != FORMIC_TRACKER.
//
// FMTRK_HOVER_FB (not MPC_THR_HOVER) is the vz-loop's hover-thrust fallback
// for whenever hover_thrust_estimate isn't valid yet (always true right at
// liftoff) - defaults to 0.70, not MPC_THR_HOVER's generic default, because
// this exact camera-carrying x500_vision SITL airframe was already measured
// at ~0.69-0.71 (see offboard_control.py's own calibration comment). Live
// SITL testing (regress9) confirmed the gap: with the generic ~0.6 default
// and the old, weaker VZ_P/I/D, the vehicle never built enough net lift to
// leave the ground before COM_DISARM_LAND kicked in.
class FormicTracker : public ModuleBase<FormicTracker>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FormicTracker();
	~FormicTracker() override;

	static int task_spawn(int argc, char *argv[]);
	static FormicTracker *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();
	void Run() override;

private:
	// What Run() did on its last tick. Every change is logged (PX4_INFO /
	// PX4_WARN, so it lands in the ulog's logged messages), to tell apart
	// the ways this module can end up publishing nothing while nav_state is
	// FORMIC_TRACKER.
	enum class TickResult : uint8_t {
		Inactive,      ///< nav_state != FORMIC_TRACKER
		EntryTick,     ///< first active tick, state reset, nothing published
		NoSetpoint,    ///< formic_tracker_setpoint copy() failed
		ZeroTimestamp, ///< formic_tracker_setpoint copied but timestamp == 0
		Published,     ///< vehicle_attitude_setpoint published
	};
	static const char *tick_result_str(TickResult result);
	void set_tick_result(TickResult result, const formic_tracker_setpoint_s *setpoint = nullptr);

	void run_once();
	void parameters_update(bool force = false);
	void reset_on_entry(float current_yaw);
	void publish_attitude_setpoint(const formic_tracker_setpoint_s &setpoint, float dt, float vz_actual);

	// --- Subscriptions ---
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _formic_tracker_setpoint_sub{ORB_ID(formic_tracker_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};

	// --- Publications ---
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	// --- State ---
	bool _was_active{false}; ///< nav_state == FORMIC_TRACKER on the previous tick
	hrt_abstime _last_run{0};
	float _yaw_setpoint{0.f}; ///< heading-hold setpoint, integrated from psi_dot_cmd
	float _hover_thrust{0.5f};

	// vz PID state (mirrors offboard_control.py's PID class)
	float _vz_integral{0.f};
	float _vz_prev_error{0.f};
	bool _vz_pid_initialized{false};

	TickResult _tick_result{TickResult::Inactive};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _active_perf{perf_alloc(PC_COUNT, MODULE_NAME": active ticks")};
	perf_counter_t _no_setpoint_perf{perf_alloc(PC_COUNT, MODULE_NAME": no setpoint")};
	perf_counter_t _zero_ts_perf{perf_alloc(PC_COUNT, MODULE_NAME": zero timestamp")};
	perf_counter_t _published_perf{perf_alloc(PC_COUNT, MODULE_NAME": published")};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FMTRK_TILT_MAX>) _param_formic_trk_tilt_max,
		(ParamFloat<px4::params::FMTRK_VZ_MAX>) _param_formic_trk_vz_max,
		(ParamFloat<px4::params::FMTRK_VZ_P>) _param_formic_trk_vz_p,
		(ParamFloat<px4::params::FMTRK_VZ_I>) _param_formic_trk_vz_i,
		(ParamFloat<px4::params::FMTRK_VZ_D>) _param_formic_trk_vz_d,
		(ParamFloat<px4::params::FMTRK_HOVER_FB>) _param_formic_trk_hover_fb
	)
};
