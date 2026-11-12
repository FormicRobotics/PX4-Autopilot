#pragma once

// PX4 platform / module framework
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>

// uORB
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/formic_vehicle_odometry.h>
#include <uORB/topics/formic_vio_est_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>

#include "OutputPropagator.hpp"
#include "VioSegment.hpp"

/**
 * Estimator instance 1: the VIO pose from formic_odom carried to now on the IMU.
 *
 * Runs on vehicle_imu instance 0, the IMU of EKF2 (instance 0), minus the biases EKF2 estimates.
 * Publishes the estimator_* topics as instance 1 for the EKF2 selector, in PX4's body frame.
 * Started at boot.
 */
class FormicVioEst : public ModuleBase<FormicVioEst>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	FormicVioEst();
	~FormicVioEst() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	void Run() override;
	int print_status() override;

	/// Bench and simulation: ignore every VIO pose for `ms`, as if the VIO stopped.
	void dropVio(uint32_t ms) { _drop_until_us.store(hrt_absolute_time() + static_cast<hrt_abstime>(ms) * 1000); }

private:
	static constexpr int IMU_HISTORY{160};          ///< 0.8 s at the 200 Hz of IMU_INTEG_RATE
	static constexpr uint8_t INSTANCE{1};

	void parameters_update(bool force = false);

	/// Advertises the estimator topics once EKF2 holds instance 0. false while waiting.
	bool advertise();

	void updateEkfInputs(const vehicle_imu_s &imu);

	/// Reads a new VIO pose, if any, and uses it.
	void updateVio(hrt_abstime now);

	/**
	 * Uses one VIO pose.
	 * @param odom          the pose; q of the FMU sensor frame when to_body, else of the body
	 * @param sample_us     its time on the PX4 clock
	 * @param clock_locked  the sample time is usable
	 */
	void useVioSample(const vehicle_odometry_s &odom, uint64_t sample_us, uint8_t status, uint64_t epoch,
			  bool clock_locked, bool to_body, hrt_abstime now);

	void countRejection(formic_vio_est::PropagatorResult r);

	/// The output jumped (new segment): published as resets.
	void recordResets(const formic_vio_est::OutputState &before, const formic_vio_est::OutputState &after);

	void updateHealth(hrt_abstime now);

	void publish(const vehicle_imu_s &imu, hrt_abstime now);
	void publishStatus(hrt_abstime now);
	void publishEstimatorStatus(const vehicle_imu_s &imu, hrt_abstime now);

	static uint8_t parseStatus(const char status[16]);

	// --- Subscriptions ---
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu), 0};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _formic_odom_sub{ORB_ID(formic_odom)};
	uORB::Subscription _visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};
	uORB::Subscription _ekf_bias_sub{ORB_ID(estimator_sensor_bias), 0};
	uORB::Subscription _ekf_local_position_sub{ORB_ID(estimator_local_position), 0};
	uORB::Subscription _ekf_status_sub{ORB_ID(estimator_status), 0};
	uORB::Subscription _ekf_status_flags_sub{ORB_ID(estimator_status_flags), 0};
	uORB::Subscription _ekf_attitude_sub{ORB_ID(estimator_attitude), 0};

	// --- Publications, all instance 1 ---
	uORB::PublicationMulti<vehicle_attitude_s> _attitude_pub{ORB_ID(estimator_attitude)};
	uORB::PublicationMulti<vehicle_local_position_s> _local_position_pub{ORB_ID(estimator_local_position)};
	uORB::PublicationMulti<vehicle_odometry_s> _odometry_pub{ORB_ID(estimator_odometry)};
	uORB::PublicationMulti<estimator_status_s> _estimator_status_pub{ORB_ID(estimator_status)};
	uORB::PublicationMulti<estimator_status_flags_s> _estimator_status_flags_pub{ORB_ID(estimator_status_flags)};
	uORB::PublicationMulti<estimator_sensor_bias_s> _estimator_sensor_bias_pub{ORB_ID(estimator_sensor_bias)};
	uORB::Publication<formic_vio_est_status_s> _status_pub{ORB_ID(formic_vio_est_status)};

	bool _advertised{false};
	hrt_abstime _start_time{0};
	bool _wait_warned{false};

	// --- Estimate ---
	formic_vio_est::OutputPropagator<IMU_HISTORY> _prop{};
	formic_vio_est::VioSegment _segment{};
	matrix::Dcmf _body_from_sensor{}; ///< level trim after board rotation, as for an internal IMU

	// --- From EKF2 (instance 0) ---
	matrix::Vector3f _gyro_bias{};
	matrix::Vector3f _accel_bias{};
	vehicle_local_position_s _ekf_local_position{};
	estimator_status_s _ekf_status{};
	estimator_status_flags_s _ekf_status_flags{};
	vehicle_attitude_s _ekf_attitude{};

	// --- VIO input ---
	px4::atomic<hrt_abstime> _drop_until_us{0};
	uint64_t _last_msg_timestamp{0};
	hrt_abstime _last_rx_us{0};            ///< arrival of the newest pose, any status
	uint64_t _last_accepted_sample_us{0};  ///< sample time of the newest pose used
	uint8_t _vio_status{formic_vio_est_status_s::VIO_STATUS_INACTIVE};
	bool _clock_locked{false};
	uint32_t _latency_us{0};
	uint32_t _latency_hist[8] {};
	formic_vio_est::TrackingError _error{};
	matrix::Vector3f _last_pos_var{NAN, NAN, NAN};
	matrix::Vector3f _last_vel_var{NAN, NAN, NAN};

	uint32_t _n_accepted{0};
	uint32_t _n_rej_status{0};
	uint32_t _n_rej_stamp{0};
	uint32_t _n_rej_old{0};
	uint32_t _n_rej_future{0};
	uint32_t _n_rej_invalid{0};

	// --- Health ---
	bool _healthy{false};
	bool _pos_offerable{false};
	bool _gap_exceeded{true};
	bool _vel_trusted{false};
	bool _health_prev{false};
	bool _offerable_prev{false};

	// --- Resets of this instance ---
	uint8_t _quat_reset_counter{0};
	matrix::Quatf _delta_q_reset{};
	uint8_t _xy_reset_counter{0};
	matrix::Vector2f _delta_xy{};
	uint8_t _z_reset_counter{0};
	float _delta_z{0.f};
	uint8_t _vxy_reset_counter{0};
	matrix::Vector2f _delta_vxy{};
	uint8_t _vz_reset_counter{0};
	float _delta_vz{0.f};
	uint8_t _heading_reset_counter{0};
	float _delta_heading{0.f};

	hrt_abstime _last_status_pub{0};
	hrt_abstime _last_estimator_status_pub{0};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::FORMIC_VIO_SRC>) _param_formic_vio_src,
		(ParamInt<px4::params::FORMIC_VIO_DLY>) _param_formic_vio_dly,     // largest pose delay, ms
		(ParamFloat<px4::params::FORMIC_VIO_TOUT>) _param_formic_vio_tout, // pose age at which the VIO is stale, s
		(ParamFloat<px4::params::FORMIC_VIO_GAP>) _param_formic_vio_gap,   // longest IMU-only gap, s
		(ParamFloat<px4::params::FORMIC_VIO_TAU_A>) _param_formic_vio_tau_a,
		(ParamFloat<px4::params::FORMIC_VIO_TAU_V>) _param_formic_vio_tau_v,
		(ParamFloat<px4::params::FORMIC_VIO_TAU_P>) _param_formic_vio_tau_p,
		(ParamFloat<px4::params::EKF2_IMU_POS_X>) _param_ekf2_imu_pos_x,
		(ParamFloat<px4::params::EKF2_IMU_POS_Y>) _param_ekf2_imu_pos_y,
		(ParamFloat<px4::params::EKF2_IMU_POS_Z>) _param_ekf2_imu_pos_z
	)
};
