#pragma once

// PX4 platform / module framework
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// uORB
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/formic_vehicle_odometry.h>
#include <uORB/topics/estimator_aid_source1d.h>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/formic_ev_flag.h>
#include <uORB/topics/formic_pos_req.h>
#include <cstring>
// Drivers
#include <drivers/drv_hrt.h>



enum class VIOStatus {
    INACTIVE,      // No estimator running. No pose.
    INITIALIZING,  // Trying to initialize. No usable estimate yet - not a failure.
    VALIDATING,    // Initialized, on probation. Poses publish but are provisional.
    NOMINAL,       // Measurements are carrying the filter.
    DEGRADED,      // Propagation is starting to dominate. Recoverable.
    DIVERGED,      // The estimate is not trustworthy in any frame.
    RESETTING,     // Tearing down and re-initializing. No pose.
};

enum class Dstate : uint8_t {
    NONE = 0,
    VisionON = 1,
    DEGRADED = 2,
    DR = 3
};


class FormicWatchdogEv : public ModuleBase<FormicWatchdogEv>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FormicWatchdogEv();
	~FormicWatchdogEv() override = default;

	static int task_spawn(int argc, char *argv[]);
	static FormicWatchdogEv *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;

private:
	// --- Methods ---
	void parameters_update(bool force = false);
	void copy_odometry_msg(vehicle_odometry_s &odometry);
	bool check_EV_aid_src_heading(float vio_yaw, float estimator_yaw); // returns true if EV yaw aid data was fused this cycle; sets _heading_alligned_with_ev
	bool resetcounter_req(vehicle_odometry_s &odometry);
	void no_EvData(); // need to get the EV data dropout status
	float get_yaw_from_quat(const vehicle_odometry_s &odometry);
	void handle_pos_req_user_intention();
	bool check_EV_aid_src_pos(const float vio_pos[2], const float estimator_pos[2]); // returns true if EV pos aid data was fused this cycle; sets pos_alligned_with_ev
	void publish_msg();
	void parse_status(const char status[16]);

	// --- Subscriptions ---
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _odometry_sub_formic{ORB_ID(formic_odom)};
	uORB::Subscription _estimator_aid_src_heading_sub{ORB_ID(estimator_aid_src_ev_yaw)};
	uORB::Subscription _estimator_aid_src_pos_sub{ORB_ID(estimator_aid_src_ev_pos)};
	uORB::Subscription _estimtor_odometry_sub{ORB_ID(vehicle_odometry)};
	uORB::Subscription _formic_pos_req{ORB_ID(formic_pos_req)};  // pos req at the tick time

	// --- Publications ---
	uORB::Publication<vehicle_odometry_s> _odometry_pub{ORB_ID(vehicle_visual_odometry)};
	uORB::Publication<formic_ev_flag_s> _formic_ev_flag_pub{ORB_ID(formic_ev_flag)};

	// --- EV stream state ---
	bool _ev_hpos_enabled{false};      // EKF2_EV_CTRL horizontal position fusion bit
	bool _ev_yaw_enabled{false};       // EKF2_EV_CTRL yaw fusion bit
	hrt_abstime _last_ev_timestamp{0}; // last EV sample arrival time (drives dropout detection)
	hrt_abstime _last_reset_time{0};  // last increment time (for the reset throttle)
	bool _heading_alligned_with_ev{false}; // true while the EKF heading is aligned with the EV yaw
	bool _pos_alligned_with_ev{false}; // true while the EKF heading is aligned with the EV yaw
	bool _ekfs_conv {false};
	bool _data_arrived{false}; // true if new EV data has arrived this cycle

	bool _pos_requested{false}; // latched formic_pos_req.pos_req: true while a position mode is requested
	int reset_counter = 0;

	Dstate _dstate{Dstate::NONE};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FORMIC_WDEV_EN>) _param_formic_wdev_en,  // use this module
		(ParamInt<px4::params::EKF2_EV_CTRL>) _param_ekf2_ev_ctrl, // EKF2 EV control bitmask
		(ParamFloat<px4::params::FORMIC_WDEV_DYAW>) _param_formic_wdev_dyaw, // maximum allowed yaw deviation
		(ParamFloat<px4::params::FORMIC_WDEV_DPOS>) _param_formic_wdev_dpos, // maximum allowed position deviation
		(ParamInt<px4::params::EKF2_NOAID_TOUT>) _param_ekf2_noaid_tout // EKF2 no-aid timeout
	)
};
