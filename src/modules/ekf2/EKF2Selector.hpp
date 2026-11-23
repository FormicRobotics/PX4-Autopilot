/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef EKF2SELECTOR_HPP
#define EKF2SELECTOR_HPP

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/time.h>
#include <lib/hysteresis/hysteresis.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/estimator_selector_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/formic_control_state.h>
#include <uORB/topics/formic_ekf_vel_reset.h>
#include <uORB/topics/formic_pos_req.h>
#include <uORB/topics/formic_source_status.h>
#include <uORB/topics/formic_vio_est_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/sensor_gps.h>

#include <lib/formic_frames/anchor.hpp>
#include <lib/formic_frames/control_state.hpp>
#include <lib/formic_frames/pose_history.hpp>
#include <lib/formic_frames/source_arbiter.hpp>

#if CONSTRAINED_MEMORY
# define EKF2_MAX_INSTANCES 2
#else
# define EKF2_MAX_INSTANCES 9
#endif

using namespace time_literals;

class EKF2Selector : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	EKF2Selector();
	~EKF2Selector() override;

	void Stop();

	void PrintStatus();

	void RequestInstance(uint8_t instance) { _request_instance.store(instance); }

private:
	static constexpr uint8_t INVALID_INSTANCE{UINT8_MAX};
	static constexpr uint64_t FILTER_UPDATE_PERIOD{10_ms};

	void Run() override;

	void PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance);

	void PublishEstimatorSelectorStatus();
	void PublishVehicleAttitude();
	void PublishVehicleLocalPosition();
	void PublishVehicleGlobalPosition();
	void PublishVehicleOdometry();
	void PublishWindEstimate();

	/// Publishes formic_control_state from `lpos`, the vehicle_local_position just published, and the last vehicle_attitude.
	void PublishControlState(const vehicle_local_position_s &lpos);

	bool SelectInstance(uint8_t instance);

	// Update the error scores for all available instances
	bool UpdateErrorScores();

	// Formic: EKF2 is instance 0 (IMU, baro, mag, GPS; no vision) and formic_vio_est is instance 1
	// (IMU, VIO; no GPS). EKF2 flies the modes without position. In a mode that needs position
	// (formic_pos_req.use_vision_now) the source arbiter picks the VIO instance or EKF2 on GPS, from GPS
	// quality, VIO health, FORMIC_EST_PREF and the RC source switch. Each instance is published through an
	// anchor (a rotation about down and a shift) that keeps the published frame.
	//
	// Without GPS the published frame is one frame per VIO segment. From the first time GPS is ready
	// (GPS good and EKF2's heading aligned) the published frame is EKF2's north with a fixed origin:
	// EKF2's anchor has no rotation, and the VIO instance is anchored onto it at every switch.
	static constexpr uint8_t FORMIC_EKF_INSTANCE{0};
	static constexpr uint8_t FORMIC_VIO_INSTANCE{1};

	void FormicRun();

	/// Runs the source arbiter on this cycle's status.
	formic_frames::ArbiterOutput FormicArbitrate(hrt_abstime now, const formic_pos_req_s &pos_req, bool vio_can_enter,
			bool vio_can_stay, bool vio_healthy);

	/**
	 * The frame rules with GPS. At the first GPS ready: north becomes EKF2's and EKF2's anchor is
	 * fitted to the published position, which does not step. After it: EKF2's heading resets are
	 * followed also while the VIO instance is published.
	 */
	void FormicUpdateFrame(bool gps_ready);

	/// Turns the published frame by `delta` about the vehicle's position, announced as resets. false: not possible now.
	bool FormicRotatePublished(float delta);

	void FormicPublishSourceStatus(hrt_abstime now, const formic_frames::ArbiterOutput &arb, uint8_t rc_position);

	/// Asks EKF2 to take the published horizontal velocity before the switch to it. false: not possible now.
	bool FormicRequestVelocitySeed();

	/**
	 * Switches the primary to `new_instance` and anchors it.
	 * @param requested a requested handover ramps the tilt; one because the VIO failed steps it
	 */
	void FormicHandover(uint8_t new_instance, bool requested, uint32_t vio_segment);

	/// Attitude of the primary as published: anchored, with the tilt ramp.
	matrix::Quatf FormicAttitudeOut(const matrix::Quatf &q, uint64_t timestamp_sample);

	void FormicPublishVehicleAttitude();
	void FormicPublishVehicleLocalPosition();
	void FormicPublishVehicleOdometry();

	// Subscriptions (per estimator instance)
	struct EstimatorInstance {

		EstimatorInstance(EKF2Selector *selector, uint8_t i) :
			estimator_attitude_sub{selector, ORB_ID(estimator_attitude), i},
			estimator_status_sub{selector, ORB_ID(estimator_status), i},
			estimator_local_position_sub{ORB_ID(estimator_local_position), i},
			estimator_global_position_sub{ORB_ID(estimator_global_position), i},
			estimator_odometry_sub{ORB_ID(estimator_odometry), i},
			estimator_wind_sub{ORB_ID(estimator_wind), i},
			instance(i)
		{
			healthy.set_hysteresis_time_from(false, 1_s);
		}

		uORB::SubscriptionCallbackWorkItem estimator_attitude_sub;
		uORB::SubscriptionCallbackWorkItem estimator_status_sub;

		uORB::Subscription estimator_local_position_sub;
		uORB::Subscription estimator_global_position_sub;
		uORB::Subscription estimator_odometry_sub;
		uORB::Subscription estimator_wind_sub;

		uint64_t timestamp_last{0};

		uint32_t accel_device_id{0};
		uint32_t gyro_device_id{0};
		uint32_t baro_device_id{0};
		uint32_t mag_device_id{0};

		hrt_abstime time_last_selected{0};
		hrt_abstime time_last_no_warning{0};

		float combined_test_ratio{NAN};
		float relative_test_ratio{NAN};

		systemlib::Hysteresis healthy{false};

		bool warning{false};
		bool filter_fault{false};
		bool timeout{false};

		uint8_t healthy_count{0};

		const uint8_t instance;
	};

	static constexpr float _rel_err_score_lim{1.0f}; // +- limit applied to the relative error score
	static constexpr float _rel_err_thresh{0.5f};    // the relative score difference needs to be greater than this to switch from an otherwise healthy instance

	EstimatorInstance _instance[EKF2_MAX_INSTANCES] {
		{this, 0},
		{this, 1},
#if EKF2_MAX_INSTANCES > 2
		{this, 2},
		{this, 3},
#if EKF2_MAX_INSTANCES > 4
		{this, 4},
		{this, 5},
		{this, 6},
		{this, 7},
		{this, 8},
#endif
#endif
	};

	static constexpr uint8_t IMU_STATUS_SIZE = (sizeof(sensors_status_imu_s::gyro_inconsistency_rad_s) / sizeof(
				sensors_status_imu_s::gyro_inconsistency_rad_s[0]));
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_gyro_error) / sizeof(
			      estimator_selector_status_s::accumulated_gyro_error[0]),
		      "increase estimator_selector_status_s::accumulated_gyro_error size");
	static_assert(IMU_STATUS_SIZE == sizeof(estimator_selector_status_s::accumulated_accel_error) / sizeof(
			      estimator_selector_status_s::accumulated_accel_error[0]),
		      "increase estimator_selector_status_s::accumulated_accel_error size");
	static_assert(EKF2_MAX_INSTANCES <= sizeof(estimator_selector_status_s::combined_test_ratio) / sizeof(
			      estimator_selector_status_s::combined_test_ratio[0]),
		      "increase estimator_selector_status_s::combined_test_ratio size");

	float _accumulated_gyro_error[IMU_STATUS_SIZE] {};
	float _accumulated_accel_error[IMU_STATUS_SIZE] {};
	hrt_abstime _last_update_us{0};
	bool _gyro_fault_detected{false};
	bool _accel_fault_detected{false};

	uint8_t _available_instances{0};
	uint8_t _selected_instance{INVALID_INSTANCE};
	px4::atomic<uint8_t> _request_instance{INVALID_INSTANCE};

	uint32_t _instance_changed_count{0};
	hrt_abstime _last_instance_change{0};

	hrt_abstime _last_status_publish{0};
	bool _selector_status_publish{false};

	// vehicle_attitude: reset counters
	vehicle_attitude_s _attitude_last{};
	vehicle_attitude_s _attitude_published{};  ///< the last vehicle_attitude as published, for formic_control_state
	matrix::Quatf _delta_q_reset{};
	uint8_t _quat_reset_counter{0};

	// vehicle_local_position: reset counters
	vehicle_local_position_s _local_position_last{};
	matrix::Vector2f _delta_xy_reset{};
	float _delta_z_reset{0.f};
	matrix::Vector2f _delta_vxy_reset{};
	float _delta_vz_reset{0.f};
	float _delta_heading_reset{0};
	float _delta_hagl_reset{0.f};

	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _heading_reset_counter{0};
	uint8_t _hagl_reset_counter{0};

	// vehicle_odometry
	vehicle_odometry_s _odometry_last{};
	uint8_t _odometry_reset_counter{0};

	// vehicle_global_position: reset counters
	vehicle_global_position_s _global_position_last{};
	double _delta_lat_reset{0};
	double _delta_lon_reset{0};
	float _delta_alt_reset{0.f};
	float _delta_terrain_reset{0.f};

	uint8_t _lat_lon_reset_counter{0};
	uint8_t _alt_reset_counter{0};
	uint8_t _terrain_reset_counter{0};

	// wind estimate
	wind_s _wind_last{};

	uint8_t _attitude_instance_prev{INVALID_INSTANCE};
	uint8_t _local_position_instance_prev{INVALID_INSTANCE};
	uint8_t _global_position_instance_prev{INVALID_INSTANCE};
	uint8_t _odometry_instance_prev{INVALID_INSTANCE};

	// Formic role policy
	formic_frames::Anchor _formic_anchor[2] {};        ///< per instance: published = anchor * instance
	bool _formic_anchor_valid[2] {true, false};       ///< EKF2 is published as it is at boot
	uint32_t _formic_anchor_segment{0};               ///< VIO segment of the instance 1 anchor
	matrix::Vector3f _formic_ramp_dtheta{};           ///< tilt removed by the ramp, earth frame, rad
	uint64_t _formic_ramp_t0{0};
	bool _formic_ramp_active{false};
	uint8_t _formic_role{FORMIC_EKF_INSTANCE};
	uint8_t _formic_override{INVALID_INSTANCE};       ///< ekf2 select_instance (bench)
	uint32_t _formic_handover_count{0};
	float _formic_unaided_offset{0.f};                 ///< added to the primary's anchored unaided heading, rad
	bool _formic_backstop_warned{false};

	// source arbiter and the GPS frame
	formic_frames::SourceArbiter _formic_arbiter{};
	formic_frames::ArbiterOutput _formic_arb{};       ///< this cycle's answer
	bool _formic_gps_ready{false};                    ///< GPS good and EKF2's heading aligned
	bool _formic_ekf_gnss{false};                     ///< EKF2 fuses GPS: no velocity seed from the VIO
	bool _formic_origin_fitted{false};                ///< north is EKF2's and its anchor shift is fixed
	uint8_t _formic_ekf_heading_counter{0};           ///< EKF2's own heading resets, followed while the VIO is published
	hrt_abstime _formic_frame_change{0};              ///< the published frame was turned: no handover right after
	hrt_abstime _formic_ref_change{0};                ///< EKF2's anchor shift changed: the published origin moved
	hrt_abstime _formic_source_status_last{0};
	formic_source_status_s _formic_source_status_prev{};
	float _formic_gps_speed{NAN};
	float _formic_vio_speed{NAN};

	bool _formic_seed_pending{false};
	hrt_abstime _formic_seed_deadline{0};
	uint8_t _formic_seed_vxy_counter{0};
	uint8_t _formic_seed_sequence{0};

	formic_frames::PoseHistory<64> _formic_att_history{};   ///< published attitude
	formic_frames::PoseHistory<64> _formic_lpos_history{};  ///< published position and velocity

	// the primary instance's own reset counters at the last publish; taken anew after a handover
	bool _formic_att_baseline{false};
	bool _formic_lpos_baseline{false};
	bool _formic_odom_baseline{false};
	uint8_t _formic_inst_quat_counter{0};
	uint8_t _formic_inst_xy_counter{0};
	uint8_t _formic_inst_z_counter{0};
	uint8_t _formic_inst_vxy_counter{0};
	uint8_t _formic_inst_vz_counter{0};
	uint8_t _formic_inst_heading_counter{0};
	uint8_t _formic_inst_hagl_counter{0};
	uint8_t _formic_inst_odom_counter{0};

	uORB::Subscription _formic_pos_req_sub{ORB_ID(formic_pos_req)};
	uORB::Subscription _formic_vio_est_status_sub{ORB_ID(formic_vio_est_status)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _formic_ekf_flags_sub{ORB_ID(estimator_status_flags), FORMIC_EKF_INSTANCE};
	// read without taking the update away from the publishers above
	uORB::Subscription _formic_lpos_sub[2] {{ORB_ID(estimator_local_position), 0}, {ORB_ID(estimator_local_position), 1}};
	uORB::Subscription _formic_att_sub[2] {{ORB_ID(estimator_attitude), 0}, {ORB_ID(estimator_attitude), 1}};
	uORB::Publication<formic_source_status_s> _formic_source_status_pub{ORB_ID(formic_source_status)};
	uORB::Publication<formic_ekf_vel_reset_s> _formic_ekf_vel_reset_pub{ORB_ID(formic_ekf_vel_reset)};
	uORB::Publication<formic_control_state_s> _formic_control_state_pub{ORB_ID(formic_control_state)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _sensors_status_imu{ORB_ID(sensors_status_imu)};

	// Publications
	uORB::Publication<estimator_selector_status_s> _estimator_selector_status_pub{ORB_ID(estimator_selector_status)};
	uORB::Publication<sensor_selection_s>          _sensor_selection_pub{ORB_ID(sensor_selection)};
	uORB::Publication<vehicle_attitude_s>          _vehicle_attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::Publication<vehicle_global_position_s>   _vehicle_global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<vehicle_local_position_s>    _vehicle_local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<vehicle_odometry_s>          _vehicle_odometry_pub{ORB_ID(vehicle_odometry)};
	uORB::Publication<wind_s>             _wind_pub{ORB_ID(wind)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::EKF2_SEL_ERR_RED>) _param_ekf2_sel_err_red,
		(ParamFloat<px4::params::EKF2_SEL_IMU_RAT>) _param_ekf2_sel_imu_angle_rate,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ANG>) _param_ekf2_sel_imu_angle,
		(ParamFloat<px4::params::EKF2_SEL_IMU_ACC>) _param_ekf2_sel_imu_accel,
		(ParamFloat<px4::params::EKF2_SEL_IMU_VEL>) _param_ekf2_sel_imu_velocity,
		(ParamFloat<px4::params::FORMIC_HO_TAU>) _param_formic_ho_tau,
		(ParamInt<px4::params::FORMIC_HO_VSEED>) _param_formic_ho_vseed,
		(ParamInt<px4::params::FORMIC_EST_PREF>) _param_formic_est_pref,
		(ParamInt<px4::params::FORMIC_GPS_SAT>) _param_formic_gps_sat,
		(ParamFloat<px4::params::FORMIC_GPS_HDOP>) _param_formic_gps_hdop,
		(ParamFloat<px4::params::FORMIC_GPS_EPH>) _param_formic_gps_eph,
		(ParamFloat<px4::params::FORMIC_GPS_TGOOD>) _param_formic_gps_tgood,
		(ParamFloat<px4::params::FORMIC_GPS_TBAD>) _param_formic_gps_tbad,
		(ParamFloat<px4::params::FORMIC_RET_DELAY>) _param_formic_ret_delay
	)
};
#endif // !EKF2SELECTOR_HPP
