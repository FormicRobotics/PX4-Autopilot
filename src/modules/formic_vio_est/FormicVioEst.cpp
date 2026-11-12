#include "FormicVioEst.hpp"

#include <cstdlib>
#include <cstring>

#include <drivers/drv_hrt.h>
#include <lib/formic_frames/board_frame.hpp>
#include <lib/formic_frames/sample_time.hpp>
#include <lib/sensor_calibration/Utilities.hpp>
#include <parameters/param.h>
#include <px4_platform_common/log.h>

using namespace time_literals;
using formic_vio_est::OutputState;
using formic_vio_est::PropagatorResult;
using formic_vio_est::SegmentAction;
using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

namespace
{
// formic_odom's own age check: a larger age, or a negative one, means the two sender stamps are
// on different clocks (formic_frames::resolve_sample_time).
constexpr int64_t AGE_VALID_US{5'000'000};

bool status_usable(uint8_t s)
{
	return s == formic_vio_est_status_s::VIO_STATUS_NOMINAL || s == formic_vio_est_status_s::VIO_STATUS_DEGRADED;
}

const char *status_name(uint8_t s)
{
	switch (s) {
	case formic_vio_est_status_s::VIO_STATUS_INACTIVE: return "INACTIVE";

	case formic_vio_est_status_s::VIO_STATUS_INITIALIZING: return "INITIALIZING";

	case formic_vio_est_status_s::VIO_STATUS_VALIDATING: return "VALIDATING";

	case formic_vio_est_status_s::VIO_STATUS_NOMINAL: return "NOMINAL";

	case formic_vio_est_status_s::VIO_STATUS_DEGRADED: return "DEGRADED";

	case formic_vio_est_status_s::VIO_STATUS_DIVERGED: return "DIVERGED";

	case formic_vio_est_status_s::VIO_STATUS_RESETTING: return "RESETTING";

	default: return "UNKNOWN";
	}
}

float tilt_between(const Quatf &a, const Quatf &b)
{
	const Vector3f za = Dcmf(a) * Vector3f(0.f, 0.f, 1.f);
	const Vector3f zb = Dcmf(b) * Vector3f(0.f, 0.f, 1.f);
	return atan2f(za.cross(zb).norm(), za.dot(zb));
}

float finite_or(float x, float fallback)
{
	return PX4_ISFINITE(x) ? x : fallback;
}
}

FormicVioEst::FormicVioEst() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
}

FormicVioEst::~FormicVioEst()
{
	perf_free(_cycle_perf);
}

bool FormicVioEst::init()
{
	_start_time = hrt_absolute_time();

	if (!_vehicle_imu_sub.registerCallback()) {
		PX4_ERR("vehicle_imu callback registration failed");
		return false;
	}

	return true;
}

void FormicVioEst::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s param_update{};
		_parameter_update_sub.copy(&param_update);
		updateParams();

		_body_from_sensor = formic_frames::body_from_sensor(calibration::GetSensorLevelAdjustment(),
				    calibration::GetBoardRotationMatrix());
		_prop.setTimeConstants(_param_formic_vio_tau_a.get(), _param_formic_vio_tau_v.get(), _param_formic_vio_tau_p.get());
	}
}

bool FormicVioEst::advertise()
{
	// EKF2 must hold instance 0 of every topic first, so these become instance 1
	const bool ekf2_present = (orb_exists(ORB_ID(estimator_attitude), 0) == PX4_OK)
				  && (orb_exists(ORB_ID(estimator_local_position), 0) == PX4_OK)
				  && (orb_exists(ORB_ID(estimator_odometry), 0) == PX4_OK)
				  && (orb_exists(ORB_ID(estimator_status), 0) == PX4_OK)
				  && (orb_exists(ORB_ID(estimator_status_flags), 0) == PX4_OK)
				  && (orb_exists(ORB_ID(estimator_sensor_bias), 0) == PX4_OK);

	if (!ekf2_present) {
		if (!_wait_warned && hrt_elapsed_time(&_start_time) > 10_s) {
			PX4_WARN("waiting for EKF2 instance 0 (SENS_IMU_MODE 0, EKF2_MULTI_IMU 1)");
			_wait_warned = true;
		}

		return false;
	}

	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_odometry_pub.advertise();
	_estimator_status_pub.advertise();
	_estimator_status_flags_pub.advertise();
	_estimator_sensor_bias_pub.advertise();

	const bool instance_ok = (_attitude_pub.get_instance() == INSTANCE)
				 && (_local_position_pub.get_instance() == INSTANCE)
				 && (_odometry_pub.get_instance() == INSTANCE)
				 && (_estimator_status_pub.get_instance() == INSTANCE)
				 && (_estimator_status_flags_pub.get_instance() == INSTANCE)
				 && (_estimator_sensor_bias_pub.get_instance() == INSTANCE);

	if (!instance_ok) {
		PX4_ERR("estimator instance %d, needs %d (att %d lpos %d odom %d status %d flags %d bias %d): "
			"is a second EKF2 instance running? EKF2_MULTI_IMU must be 1. Stopping.",
			_estimator_status_pub.get_instance(), INSTANCE, _attitude_pub.get_instance(),
			_local_position_pub.get_instance(), _odometry_pub.get_instance(), _estimator_status_pub.get_instance(),
			_estimator_status_flags_pub.get_instance(), _estimator_sensor_bias_pub.get_instance());
		request_stop();
		return false;
	}

	PX4_INFO("publishing as estimator instance %d", INSTANCE);
	_advertised = true;
	return true;
}

void FormicVioEst::Run()
{
	if (should_exit()) {
		_vehicle_imu_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	parameters_update();

	vehicle_imu_s imu;

	if ((_advertised || advertise()) && _vehicle_imu_sub.update(&imu)) {
		const hrt_abstime now = hrt_absolute_time();

		updateEkfInputs(imu);

		formic_vio_est::ImuDelta delta;
		delta.time_us = imu.timestamp_sample;
		const float dt_ang = imu.delta_angle_dt * 1e-6f;
		const float dt_vel = imu.delta_velocity_dt * 1e-6f;
		delta.dt = dt_vel;
		delta.delta_ang = Vector3f(imu.delta_angle) - _gyro_bias * dt_ang;
		delta.delta_vel = Vector3f(imu.delta_velocity) - _accel_bias * dt_vel;

		_prop.update(delta);

		updateVio(now);
		updateHealth(now);
		publish(imu, now);
	}

	perf_end(_cycle_perf);
}

void FormicVioEst::updateEkfInputs(const vehicle_imu_s &imu)
{
	estimator_sensor_bias_s bias;

	if (_ekf_bias_sub.update(&bias)) {
		_gyro_bias = (bias.gyro_device_id == imu.gyro_device_id) ? Vector3f(bias.gyro_bias) : Vector3f{};
		_accel_bias = (bias.accel_device_id == imu.accel_device_id) ? Vector3f(bias.accel_bias) : Vector3f{};

		// The rate loop and the commander checks read the primary's bias. The same estimate from two
		// instances must not count twice in the in-flight calibration (VehicleIMU), which only learns
		// from a stable one.
		bias.gyro_bias_stable = false;
		bias.accel_bias_stable = false;
		bias.mag_bias_stable = false;
		_estimator_sensor_bias_pub.publish(bias);
	}

	_ekf_local_position_sub.update(&_ekf_local_position);
	_ekf_status_sub.update(&_ekf_status);
	_ekf_status_flags_sub.update(&_ekf_status_flags);
	_ekf_attitude_sub.update(&_ekf_attitude);
}

uint8_t FormicVioEst::parseStatus(const char status[16])
{
	struct Name {
		const char *name;
		uint8_t value;
	};

	static constexpr Name names[] {
		{"INACTIVE", formic_vio_est_status_s::VIO_STATUS_INACTIVE},
		{"INITIALIZING", formic_vio_est_status_s::VIO_STATUS_INITIALIZING},
		{"VALIDATING", formic_vio_est_status_s::VIO_STATUS_VALIDATING},
		{"NOMINAL", formic_vio_est_status_s::VIO_STATUS_NOMINAL},
		{"DEGRADED", formic_vio_est_status_s::VIO_STATUS_DEGRADED},
		{"DIVERGED", formic_vio_est_status_s::VIO_STATUS_DIVERGED},
		{"RESETTING", formic_vio_est_status_s::VIO_STATUS_RESETTING},
	};

	for (const Name &n : names) {
		if (strncmp(status, n.name, 16) == 0) {
			return n.value;
		}
	}

	return formic_vio_est_status_s::VIO_STATUS_UNKNOWN;
}

void FormicVioEst::updateVio(hrt_abstime now)
{
	if (now < _drop_until_us.load()) {
		// formic_vio_est drop: the poses are read and thrown away
		vehicle_odometry_s odom;
		formic_vehicle_odometry_s formic;
		_visual_odometry_sub.update(&odom);
		_formic_odom_sub.update(&formic);
		return;
	}

	if (_param_formic_vio_src.get() == 1) {
		// Gazebo: the pose of the body, stamped on the PX4 clock
		vehicle_odometry_s odom;

		if (_visual_odometry_sub.update(&odom)) {
			const uint8_t status = (odom.quality < 0) ? formic_vio_est_status_s::VIO_STATUS_DIVERGED
					       : formic_vio_est_status_s::VIO_STATUS_NOMINAL;
			useVioSample(odom, odom.timestamp_sample, status, odom.reset_counter, true, false, now);
		}

		return;
	}

	formic_vehicle_odometry_s formic;

	if (_formic_odom_sub.update(&formic) && (formic.timestamp != _last_msg_timestamp)) {
		_last_msg_timestamp = formic.timestamp;

		// the publish time on the PX4 clock minus the sender's age
		const auto sample = formic_frames::resolve_sample_time(formic.odometry.timestamp, formic.odometry.timestamp_sample,
				    formic.timestamp, now, static_cast<int64_t>(_param_formic_vio_dly.get()) * 1000, AGE_VALID_US);

		const bool clock_locked = (sample.source != formic_frames::StampSource::POLL_TIME);
		uint64_t sample_us = sample.time_us;

		if (sample.source == formic_frames::StampSource::STALE) {
			// keep the true sample time so the age check below drops it
			sample_us = (formic.timestamp > static_cast<uint64_t>(sample.age_us)) ? formic.timestamp - sample.age_us : 0;
		}

		useVioSample(formic.odometry, sample_us, parseStatus(formic.status), formic.epoch, clock_locked, true, now);
	}
}

void FormicVioEst::useVioSample(const vehicle_odometry_s &odom, uint64_t sample_us, uint8_t status, uint64_t epoch,
				bool clock_locked, bool to_body, hrt_abstime now)
{
	_last_rx_us = now;
	_vio_status = status;
	_clock_locked = clock_locked;

	if (!clock_locked) {
		_n_rej_stamp++;
		return;
	}

	const int64_t latency_us = static_cast<int64_t>(now) - static_cast<int64_t>(sample_us);
	_latency_us = (latency_us > 0) ? static_cast<uint32_t>(latency_us) : 0;

	if (sample_us == 0 || latency_us > static_cast<int64_t>(_param_formic_vio_dly.get()) * 1000) {
		_n_rej_old++;
		return;
	}

	if (!status_usable(status)) {
		_n_rej_status++;
		return;
	}

	// --- frame ---
	const bool pose_frame_ok = (odom.pose_frame == vehicle_odometry_s::POSE_FRAME_NED)
				   || (odom.pose_frame == vehicle_odometry_s::POSE_FRAME_FRD);
	const Vector3f p(odom.position);
	Quatf q_in(odom.q);

	if (!pose_frame_ok || !p.isAllFinite() || !q_in.isAllFinite() || q_in.length() < 0.9f) {
		_n_rej_invalid++;
		return;
	}

	q_in.normalize();

	Vector3f v(odom.velocity);
	bool vel_valid = v.isAllFinite();

	if (odom.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD) {
		v = Dcmf(q_in) * v; // the frame of q_in: the sensor frame for formic_odom

	} else if (odom.velocity_frame != vehicle_odometry_s::VELOCITY_FRAME_NED
		   && odom.velocity_frame != vehicle_odometry_s::VELOCITY_FRAME_FRD) {
		vel_valid = false;
	}

	// The pose is of the FMU IMU: the position and velocity are those of the IMU point, which the
	// output carries. Only the orientation is turned from the sensor frame into the body frame.
	Quatf q = q_in;

	if (to_body) {
		float q_arr[4] = {q_in(0), q_in(1), q_in(2), q_in(3)};

		if (!formic_frames::sensor_to_body_orientation(q_arr, _body_from_sensor)) {
			_n_rej_invalid++;
			return;
		}

		q = Quatf(q_arr);
	}

	// --- use it ---
	const uint64_t gap_max_us = static_cast<uint64_t>(_param_formic_vio_gap.get() * 1e6f);
	PropagatorResult r = PropagatorResult::ACCEPTED;

	const SegmentAction action = _segment.classify(epoch, sample_us, gap_max_us);

	if (action == SegmentAction::NEW_SEGMENT) {
		if (!vel_valid) {
			_n_rej_invalid++;
			return;
		}

		const bool had_output = _prop.initialised();
		const OutputState before = _prop.output();
		r = _prop.initialise(sample_us, q, v, p);

		if (r != PropagatorResult::ACCEPTED) {
			countRejection(r);
			return;
		}

		_segment.startSegment(epoch, sample_us);
		_error = formic_vio_est::TrackingError{};

		if (had_output) {
			recordResets(before, _prop.output());
		}

		PX4_INFO("VIO segment %" PRIu32 " started, epoch %" PRIu64, _segment.id(), epoch);

	} else {
		if (action == SegmentAction::NEW_EPOCH) {
			// a VIO restart within the gap: anchor its frame onto the output the IMU carried
			OutputState s;
			r = _prop.stateAt(sample_us, s);

			if (r != PropagatorResult::ACCEPTED) {
				countRejection(r);
				return;
			}

			_segment.anchorEpoch(epoch, sample_us, s.q, s.p, q, p);
			PX4_INFO("VIO epoch %" PRIu64 " anchored onto segment %" PRIu32, epoch, _segment.id());
		}

		const formic_frames::Anchor &a = _segment.epochAnchor();
		r = _prop.correct(sample_us, a.apply(q), a.applyVector(v), vel_valid, a.applyPosition(p), &_error);

		if (r != PropagatorResult::ACCEPTED) {
			countRejection(r);
			return;
		}

		_segment.used(sample_us);
	}

	_n_accepted++;
	_last_accepted_sample_us = sample_us;
	_latency_hist[math::min(_latency_us / 50000u, static_cast<uint32_t>(7))]++;
	_last_pos_var = Vector3f(odom.position_variance);
	_last_vel_var = Vector3f(odom.velocity_variance);
}

void FormicVioEst::countRejection(PropagatorResult r)
{
	if (r == PropagatorResult::IN_FUTURE) {
		_n_rej_future++;

	} else {
		_n_rej_old++;
	}
}

void FormicVioEst::recordResets(const OutputState &before, const OutputState &after)
{
	_delta_q_reset = Quatf((after.q * before.q.inversed()).normalized());
	_quat_reset_counter++;
	_delta_xy = Vector2f(after.p(0) - before.p(0), after.p(1) - before.p(1));
	_xy_reset_counter++;
	_delta_z = after.p(2) - before.p(2);
	_z_reset_counter++;
	_delta_vxy = Vector2f(after.v(0) - before.v(0), after.v(1) - before.v(1));
	_vxy_reset_counter++;
	_delta_vz = after.v(2) - before.v(2);
	_vz_reset_counter++;
	_delta_heading = matrix::wrap_pi(formic_frames::yaw_of(after.q) - formic_frames::yaw_of(before.q));
	_heading_reset_counter++;
}

void FormicVioEst::updateHealth(hrt_abstime now)
{
	const bool initialised = _prop.initialised();
	// a pose can be stamped at or a little after now (no latency in simulation): age 0
	uint64_t age_us = UINT64_MAX;

	if (initialised) {
		age_us = (now > _last_accepted_sample_us) ? now - _last_accepted_sample_us : 0;
	}

	const uint64_t tout_us = static_cast<uint64_t>(_param_formic_vio_tout.get() * 1e6f);
	const uint64_t gap_us = static_cast<uint64_t>(_param_formic_vio_gap.get() * 1e6f);

	_healthy = initialised && (age_us < tout_us) && status_usable(_vio_status) && _clock_locked
		   && (now - _last_rx_us < tout_us);
	_pos_offerable = _healthy && (_vio_status == formic_vio_est_status_s::VIO_STATUS_NOMINAL);
	_gap_exceeded = !initialised || (age_us > gap_us);
	_vel_trusted = !_gap_exceeded && (_vio_status != formic_vio_est_status_s::VIO_STATUS_DIVERGED);

	if (_gap_exceeded && _segment.active()) {
		_segment.end();
		PX4_WARN("VIO gap over FORMIC_VIO_GAP: segment %" PRIu32 " ended", _segment.id());
	}
}

void FormicVioEst::publish(const vehicle_imu_s &imu, hrt_abstime now)
{
	if (now - _last_estimator_status_pub >= 10_ms) {
		publishEstimatorStatus(imu, now);
	}

	if (now - _last_status_pub >= 50_ms || _healthy != _health_prev || _pos_offerable != _offerable_prev) {
		publishStatus(now);
		_health_prev = _healthy;
		_offerable_prev = _pos_offerable;
	}

	if (!_prop.initialised()) {
		return;
	}

	const OutputState &out = _prop.output();
	const Dcmf R(out.q);
	const Vector3f rate = _prop.angularRate();
	const Vector3f imu_pos(_param_ekf2_imu_pos_x.get(), _param_ekf2_imu_pos_y.get(), _param_ekf2_imu_pos_z.get());

	// the body origin, as EKF2's output predictor publishes it
	const Vector3f p = out.p - R * imu_pos;
	const Vector3f v = out.v - R * rate.cross(imu_pos);
	const Vector3f a = _prop.acceleration();
	const float heading = formic_frames::yaw_of(out.q);

	// Valid while the IMU carries the output through a VIO gap, up to FORMIC_VIO_GAP: Position mode keeps
	// flying on it until commander has changed the mode (commander fails over on the VIO health instead).
	const bool valid = !_gap_exceeded;
	const bool dead_reckoning = !_healthy;

	const float pos_sd = sqrtf(fmaxf(finite_or(_last_pos_var(0) + _last_pos_var(1), 0.01f), 1e-4f));
	const float pos_d_sd = sqrtf(fmaxf(finite_or(_last_pos_var(2), 0.01f), 1e-4f));
	const float vel_sd = sqrtf(fmaxf(finite_or(_last_vel_var(0) + _last_vel_var(1), 0.01f), 1e-4f));
	const float vel_d_sd = sqrtf(fmaxf(finite_or(_last_vel_var(2), 0.01f), 1e-4f));

	// estimator_attitude
	{
		vehicle_attitude_s att{};
		att.timestamp_sample = out.time_us;
		out.q.copyTo(att.q);
		_delta_q_reset.copyTo(att.delta_q_reset);
		att.quat_reset_counter = _quat_reset_counter;
		att.timestamp = hrt_absolute_time();
		_attitude_pub.publish(att);
	}

	// estimator_local_position
	{
		vehicle_local_position_s lpos = _ekf_local_position; // height above ground, limits and reference from EKF2
		lpos.timestamp_sample = out.time_us;

		lpos.xy_valid = valid;
		lpos.z_valid = valid;
		lpos.v_xy_valid = valid;
		lpos.v_z_valid = valid;

		lpos.x = p(0);
		lpos.y = p(1);
		lpos.z = p(2);
		_delta_xy.copyTo(lpos.delta_xy);
		lpos.xy_reset_counter = _xy_reset_counter;
		lpos.delta_z = _delta_z;
		lpos.z_reset_counter = _z_reset_counter;

		lpos.vx = v(0);
		lpos.vy = v(1);
		lpos.vz = v(2);
		lpos.z_deriv = v(2);
		_delta_vxy.copyTo(lpos.delta_vxy);
		lpos.vxy_reset_counter = _vxy_reset_counter;
		lpos.delta_vz = _delta_vz;
		lpos.vz_reset_counter = _vz_reset_counter;

		lpos.ax = a(0);
		lpos.ay = a(1);
		lpos.az = a(2);

		lpos.heading = heading;
		lpos.heading_var = 1e-4f;
		lpos.unaided_heading = heading;
		lpos.delta_heading = _delta_heading;
		lpos.heading_reset_counter = _heading_reset_counter;
		lpos.heading_good_for_control = valid;
		lpos.tilt_var = 1e-5f;

		lpos.xy_global = false;
		lpos.z_global = false;

		lpos.eph = pos_sd;
		lpos.epv = pos_d_sd;
		lpos.evh = vel_sd;
		lpos.evv = vel_d_sd;
		lpos.dead_reckoning = dead_reckoning;

		lpos.timestamp = hrt_absolute_time();
		_local_position_pub.publish(lpos);
	}

	// estimator_odometry
	{
		vehicle_odometry_s odom{};
		odom.timestamp_sample = out.time_us;
		odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
		p.copyTo(odom.position);
		out.q.copyTo(odom.q);
		odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
		v.copyTo(odom.velocity);
		rate.copyTo(odom.angular_velocity);

		for (int i = 0; i < 3; i++) {
			odom.position_variance[i] = (i < 2) ? 0.5f * pos_sd * pos_sd : pos_d_sd * pos_d_sd;
			odom.orientation_variance[i] = 1e-4f;
			odom.velocity_variance[i] = (i < 2) ? 0.5f * vel_sd * vel_sd : vel_d_sd * vel_d_sd;
		}

		odom.reset_counter = _xy_reset_counter + _vxy_reset_counter + _quat_reset_counter;
		odom.quality = _healthy ? 100 : 0;
		odom.timestamp = hrt_absolute_time();
		_odometry_pub.publish(odom);
	}
}

void FormicVioEst::publishEstimatorStatus(const vehicle_imu_s &imu, hrt_abstime now)
{
	const bool initialised = _prop.initialised();

	estimator_status_s status{};
	status.timestamp_sample = initialised ? _prop.output().time_us : imu.timestamp_sample;
	status.output_tracking_error[0] = _error.att;
	status.output_tracking_error[1] = _error.vel;
	status.output_tracking_error[2] = _error.pos;

	uint64_t flags = 0;

	if (initialised) {
		flags |= (1ull << estimator_status_s::CS_TILT_ALIGN) | (1ull << estimator_status_s::CS_YAW_ALIGN);
	}

	if (_healthy) {
		flags |= (1ull << estimator_status_s::CS_EV_POS) | (1ull << estimator_status_s::CS_EV_YAW)
			 | (1ull << estimator_status_s::CS_EV_VEL);
	}

	flags |= _ekf_status.control_mode_flags & (1ull << estimator_status_s::CS_IN_AIR);
	status.control_mode_flags = flags;

	// The selector takes an instance with a filter fault as unhealthy: before the first pose and
	// after FORMIC_VIO_GAP without one.
	status.filter_fault_flags = _gap_exceeded ? 1 : 0;

	status.pos_horiz_accuracy = sqrtf(fmaxf(finite_or(_last_pos_var(0) + _last_pos_var(1), 0.01f), 1e-4f));
	status.pos_vert_accuracy = sqrtf(fmaxf(finite_or(_last_pos_var(2), 0.01f), 1e-4f));

	// the selector's error score: low while the VIO carries the output, at the warning level when stale
	const float ratio = _healthy ? 0.1f : 1.f;
	status.hdg_test_ratio = ratio;
	status.vel_test_ratio = ratio;
	status.pos_test_ratio = ratio;
	status.hgt_test_ratio = ratio;

	status.reset_count_vel_ne = _vxy_reset_counter;
	status.reset_count_vel_d = _vz_reset_counter;
	status.reset_count_pos_ne = _xy_reset_counter;
	status.reset_count_pod_d = _z_reset_counter;
	status.reset_count_quat = _quat_reset_counter;

	// the rate loop takes its gyro from the device IDs of the primary instance
	status.accel_device_id = imu.accel_device_id;
	status.gyro_device_id = imu.gyro_device_id;
	status.baro_device_id = _ekf_status.baro_device_id;
	status.mag_device_id = _ekf_status.mag_device_id;

	status.timestamp = hrt_absolute_time();
	_estimator_status_pub.publish(status);
	_last_estimator_status_pub = now;

	estimator_status_flags_s flags_msg = _ekf_status_flags;
	flags_msg.timestamp_sample = status.timestamp_sample;
	flags_msg.cs_tilt_align = initialised;
	flags_msg.cs_yaw_align = initialised;
	flags_msg.cs_ev_pos = _healthy;
	flags_msg.cs_ev_yaw = _healthy;
	flags_msg.cs_ev_vel = _healthy;
	flags_msg.cs_inertial_dead_reckoning = initialised && !_healthy;
	flags_msg.cs_gnss_pos = false;
	flags_msg.cs_opt_flow = false;
	flags_msg.cs_fake_pos = false;
	flags_msg.cs_valid_fake_pos = false;
	flags_msg.timestamp = hrt_absolute_time();
	_estimator_status_flags_pub.publish(flags_msg);
}

void FormicVioEst::publishStatus(hrt_abstime now)
{
	formic_vio_est_status_s s{};
	s.timestamp_sample = _last_accepted_sample_us;
	s.initialised = _prop.initialised();
	s.healthy = _healthy;
	s.pos_offerable = _pos_offerable;
	s.gap_exceeded = _gap_exceeded;
	s.vel_trusted = _vel_trusted;
	s.clock_locked = _clock_locked;
	s.vio_status = _vio_status;
	s.epoch = _segment.epoch();
	s.segment = _segment.id();
	s.epoch_changes = _segment.epochChanges();
	s.epoch_anchor_yaw = _segment.epochAnchor().psi;
	_segment.epochAnchor().t.copyTo(s.epoch_anchor_shift);
	if (_last_accepted_sample_us == 0) {
		s.sample_age_us = UINT32_MAX;

	} else {
		s.sample_age_us = (now > _last_accepted_sample_us)
				  ? static_cast<uint32_t>(math::min(now - _last_accepted_sample_us, (uint64_t)UINT32_MAX)) : 0;
	}

	s.latency_us = _latency_us;

	for (int i = 0; i < 8; i++) {
		s.latency_hist[i] = _latency_hist[i];
	}

	s.err_att = _error.att;
	s.err_vel = _error.vel;
	s.err_pos = _error.pos;
	s.tilt_diff_ekf = (s.initialised && _ekf_attitude.timestamp != 0)
			  ? tilt_between(_prop.output().q, Quatf(_ekf_attitude.q)) : NAN;

	s.n_accepted = _n_accepted;
	s.n_rej_status = _n_rej_status;
	s.n_rej_stamp = _n_rej_stamp;
	s.n_rej_old = _n_rej_old;
	s.n_rej_future = _n_rej_future;
	s.n_rej_invalid = _n_rej_invalid;

	s.timestamp = hrt_absolute_time();
	_status_pub.publish(s);
	_last_status_pub = now;
}

int FormicVioEst::print_status()
{
	const hrt_abstime now = hrt_absolute_time();
	PX4_INFO("instance %d: %s, input %s", INSTANCE, _advertised ? "publishing" : "waiting for EKF2",
		 _param_formic_vio_src.get() == 1 ? "vehicle_visual_odometry" : "formic_odom");
	PX4_INFO("output %s, healthy %d, offerable %d, gap exceeded %d, velocity trusted %d",
		 _prop.initialised() ? "running" : "not started", _healthy, _pos_offerable, _gap_exceeded, _vel_trusted);
	PX4_INFO("VIO status %s, clock %s, segment %" PRIu32 " (epoch %" PRIu64 ", %" PRIu32 " restarts anchored)",
		 status_name(_vio_status), _clock_locked ? "locked" : "not locked", _segment.id(), _segment.epoch(),
		 _segment.epochChanges());

	if (_last_accepted_sample_us > 0) {
		PX4_INFO("newest pose used %.1f ms ago, arrived %.1f ms late", (double)(now - _last_accepted_sample_us) * 1e-3,
			 (double)_latency_us * 1e-3);
	}

	PX4_INFO("latency 0-50-100-150-200-250-300-350+ ms: %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %"
		 PRIu32 " %" PRIu32 " %" PRIu32, _latency_hist[0], _latency_hist[1], _latency_hist[2], _latency_hist[3],
		 _latency_hist[4], _latency_hist[5], _latency_hist[6], _latency_hist[7]);
	PX4_INFO("poses used %" PRIu32 "; dropped: status %" PRIu32 ", stamps %" PRIu32 ", old %" PRIu32 ", future %" PRIu32
		 ", invalid %" PRIu32, _n_accepted, _n_rej_status, _n_rej_stamp, _n_rej_old, _n_rej_future, _n_rej_invalid);
	PX4_INFO("tracking error: attitude %.4f rad, velocity %.3f m/s, position %.3f m", (double)_error.att,
		 (double)_error.vel, (double)_error.pos);

	if (_prop.initialised() && _ekf_attitude.timestamp != 0) {
		PX4_INFO("tilt to EKF2: %.2f deg", (double)math::degrees(tilt_between(_prop.output().q, Quatf(_ekf_attitude.q))));
	}

	perf_print_counter(_cycle_perf);
	return PX4_OK;
}

int FormicVioEst::task_spawn(int argc, char *argv[])
{
	FormicVioEst *instance = new FormicVioEst();

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

int FormicVioEst::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		return print_usage("not running");
	}

	if (argc >= 2 && strcmp(argv[0], "drop") == 0) {
		const int ms = atoi(argv[1]);

		if (ms > 0) {
			get_instance()->dropVio(static_cast<uint32_t>(ms));
			PX4_INFO("ignoring VIO poses for %d ms", ms);
			return PX4_OK;
		}
	}

	return print_usage("unknown command");
}

int FormicVioEst::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Estimator instance 1: the VIO pose from formic_odom carried to now on the IMU.
EKF2 runs as instance 0 and never sees vision.

The pose of the FMU IMU arrives 70-125 ms after its sample time. The module integrates
vehicle_imu instance 0, minus EKF2's bias estimates, from the sample time to now and pulls the
output toward each pose (FORMIC_VIO_TAU_A/V/P). It publishes the body origin in PX4's body frame
(SENS_BOARD_ROT, SENS_BOARD_X/Y/Z_OFF, EKF2_IMU_POS_X/Y/Z).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_vio_est", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("drop", "Bench and simulation: ignore VIO poses, as if the VIO stopped");
	PRINT_MODULE_USAGE_ARG("<ms>", "how long", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int formic_vio_est_main(int argc, char *argv[])
{
	return FormicVioEst::main(argc, argv);
}
