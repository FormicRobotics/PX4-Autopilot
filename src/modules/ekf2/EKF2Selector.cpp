/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "EKF2Selector.hpp"

using namespace time_literals;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;
using math::constrain;
using math::radians;

EKF2Selector::EKF2Selector() :
	ModuleParams(nullptr),
	ScheduledWorkItem("ekf2_selector", px4::wq_configurations::nav_and_controllers)
{
	_estimator_selector_status_pub.advertise();
	_sensor_selection_pub.advertise();
	_vehicle_attitude_pub.advertise();
	_vehicle_global_position_pub.advertise();
	_vehicle_local_position_pub.advertise();
	_vehicle_odometry_pub.advertise();
	_wind_pub.advertise();

	_formic_mode = (_param_formic_est_mode.get() == 1);

	if (_formic_mode) {
		PX4_INFO("FORMIC_EST_MODE 1: role policy, EKF2 instance %d, VIO instance %d", FORMIC_EKF_INSTANCE,
			 FORMIC_VIO_INSTANCE);
	}
}

EKF2Selector::~EKF2Selector()
{
	Stop();
}

void EKF2Selector::Stop()
{
	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		_instance[i].estimator_attitude_sub.unregisterCallback();
		_instance[i].estimator_status_sub.unregisterCallback();
	}

	ScheduleClear();
}

void EKF2Selector::PrintInstanceChange(const uint8_t old_instance, uint8_t new_instance)
{
	const char *old_reason = nullptr;

	if (_instance[old_instance].filter_fault) {
		old_reason = " (filter fault)";

	} else if (_instance[old_instance].timeout) {
		old_reason = " (timeout)";

	} else if (_gyro_fault_detected) {
		old_reason = " (gyro fault)";

	} else if (_accel_fault_detected) {
		old_reason = " (accel fault)";

	} else if (!_instance[_selected_instance].healthy.get_state() && (_instance[_selected_instance].healthy_count > 0)) {
		// skipped if previous instance was never healthy in the first place (eg initialization)
		old_reason = " (unhealthy)";
	}

	const char *new_reason = nullptr;

	if (_request_instance.load() == new_instance) {
		new_reason = " (user selected)";
	}

	if (old_reason || new_reason) {
		if (old_reason == nullptr) {
			old_reason = "";
		}

		if (new_reason == nullptr) {
			new_reason = "";
		}

		PX4_WARN("primary EKF changed %" PRIu8 "%s -> %" PRIu8 "%s", old_instance, old_reason, new_instance, new_reason);
	}
}

bool EKF2Selector::SelectInstance(uint8_t ekf_instance)
{
	if ((ekf_instance != _selected_instance) && (ekf_instance < _available_instances)) {
		// update sensor_selection immediately
		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = _instance[ekf_instance].accel_device_id;
		sensor_selection.gyro_device_id = _instance[ekf_instance].gyro_device_id;
		sensor_selection.timestamp = hrt_absolute_time();
		_sensor_selection_pub.publish(sensor_selection);

		if (_selected_instance != INVALID_INSTANCE) {
			// switch callback registration
			_instance[_selected_instance].estimator_attitude_sub.unregisterCallback();
			_instance[_selected_instance].estimator_status_sub.unregisterCallback();

			PrintInstanceChange(_selected_instance, ekf_instance);
		}

		_instance[ekf_instance].estimator_attitude_sub.registerCallback();
		_instance[ekf_instance].estimator_status_sub.registerCallback();

		_selected_instance = ekf_instance;
		_instance_changed_count++;
		_last_instance_change = sensor_selection.timestamp;
		_instance[ekf_instance].time_last_selected = _last_instance_change;

		// reset all relative test ratios
		for (uint8_t i = 0; i < _available_instances; i++) {
			_instance[i].relative_test_ratio = 0;
		}

		return true;
	}

	return false;
}

bool EKF2Selector::UpdateErrorScores()
{
	// first check imu inconsistencies
	_gyro_fault_detected = false;
	uint32_t faulty_gyro_id = 0;
	_accel_fault_detected = false;
	uint32_t faulty_accel_id = 0;

	if (_sensors_status_imu.updated()) {
		sensors_status_imu_s sensors_status_imu;

		if (_sensors_status_imu.copy(&sensors_status_imu)) {

			const float time_step_s = constrain((sensors_status_imu.timestamp - _last_update_us) * 1e-6f, 0.f, 0.02f);
			_last_update_us = sensors_status_imu.timestamp;

			{
				const float angle_rate_threshold = radians(_param_ekf2_sel_imu_angle_rate.get());
				const float angle_threshold = radians(_param_ekf2_sel_imu_angle.get());
				uint8_t n_gyros = 0;
				uint8_t n_gyro_exceedances = 0;
				float largest_accumulated_gyro_error = 0.0f;
				uint8_t largest_gyro_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for gyros with excessive difference to mean using accumulated error
					if (sensors_status_imu.gyro_device_ids[i] != 0) {
						n_gyros++;
						_accumulated_gyro_error[i] += (sensors_status_imu.gyro_inconsistency_rad_s[i] - angle_rate_threshold) * time_step_s;
						_accumulated_gyro_error[i] = fmaxf(_accumulated_gyro_error[i], 0.f);

						if (_accumulated_gyro_error[i] > angle_threshold) {
							n_gyro_exceedances++;
						}

						if (_accumulated_gyro_error[i] > largest_accumulated_gyro_error) {
							largest_accumulated_gyro_error = _accumulated_gyro_error[i];
							largest_gyro_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_gyro_error[i] = NAN;
					}
				}

				if (n_gyro_exceedances > 0) {
					if (n_gyros >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_gyro_fault_detected = true;
						faulty_gyro_id = sensors_status_imu.gyro_device_ids[largest_gyro_error_index];

					} else if (n_gyros == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_gyro_fault_detected = true;
					}
				}
			}

			{
				const float accel_threshold = _param_ekf2_sel_imu_accel.get();
				const float velocity_threshold = _param_ekf2_sel_imu_velocity.get();
				uint8_t n_accels = 0;
				uint8_t n_accel_exceedances = 0;
				float largest_accumulated_accel_error = 0.0f;
				uint8_t largest_accel_error_index = 0;

				for (unsigned i = 0; i < IMU_STATUS_SIZE; i++) {
					// check for accelerometers with excessive difference to mean using accumulated error
					if (sensors_status_imu.accel_device_ids[i] != 0) {
						n_accels++;
						_accumulated_accel_error[i] += (sensors_status_imu.accel_inconsistency_m_s_s[i] - accel_threshold) * time_step_s;
						_accumulated_accel_error[i] = fmaxf(_accumulated_accel_error[i], 0.f);

						if (_accumulated_accel_error[i] > velocity_threshold) {
							n_accel_exceedances++;
						}

						if (_accumulated_accel_error[i] > largest_accumulated_accel_error) {
							largest_accumulated_accel_error = _accumulated_accel_error[i];
							largest_accel_error_index = i;
						}

					} else {
						// no sensor
						_accumulated_accel_error[i] = NAN;
					}
				}

				if (n_accel_exceedances > 0) {
					if (n_accels >= 3) {
						// If there are 3 or more sensors, the one with the largest accumulated error is faulty
						_accel_fault_detected = true;
						faulty_accel_id = sensors_status_imu.accel_device_ids[largest_accel_error_index];

					} else if (n_accels == 2) {
						// A fault is present, but the faulty sensor identity cannot be determined
						_accel_fault_detected = true;
					}
				}
			}
		}
	}

	bool updated = false;
	bool primary_updated = false;

	// default estimator timeout
	const hrt_abstime status_timeout = 50_ms;

	// calculate individual error scores
	for (uint8_t i = 0; i < EKF2_MAX_INSTANCES; i++) {
		const bool prev_healthy = _instance[i].healthy.get_state();

		estimator_status_s status;

		if (_instance[i].estimator_status_sub.update(&status)) {

			_instance[i].timestamp_last = status.timestamp;

			_instance[i].accel_device_id = status.accel_device_id;
			_instance[i].gyro_device_id = status.gyro_device_id;
			_instance[i].baro_device_id = status.baro_device_id;
			_instance[i].mag_device_id = status.mag_device_id;

			if ((i + 1) > _available_instances) {
				_available_instances = i + 1;
				updated = true;
			}

			if (i == _selected_instance) {
				primary_updated = true;
			}

			// test ratios are invalid when 0, >= 1 is a failure
			if (!PX4_ISFINITE(status.vel_test_ratio) || (status.vel_test_ratio <= 0.f)) {
				status.vel_test_ratio = 1.f;
			}

			if (!PX4_ISFINITE(status.pos_test_ratio) || (status.pos_test_ratio <= 0.f)) {
				status.pos_test_ratio = 1.f;
			}

			if (!PX4_ISFINITE(status.hgt_test_ratio) || (status.hgt_test_ratio <= 0.f)) {
				status.hgt_test_ratio = 1.f;
			}

			float combined_test_ratio = fmaxf(0.5f * (status.vel_test_ratio + status.pos_test_ratio), status.hgt_test_ratio);

			_instance[i].combined_test_ratio = combined_test_ratio;

			const bool healthy = (status.filter_fault_flags == 0) && (combined_test_ratio > 0.f);
			_instance[i].healthy.set_state_and_update(healthy, status.timestamp);

			_instance[i].warning = (combined_test_ratio >= 1.f);
			_instance[i].filter_fault = (status.filter_fault_flags != 0);
			_instance[i].timeout = false;

			if (!_instance[i].warning) {
				_instance[i].time_last_no_warning = status.timestamp;
			}

			if (!PX4_ISFINITE(_instance[i].relative_test_ratio)) {
				_instance[i].relative_test_ratio = 0;
			}

		} else if (!_instance[i].timeout && (hrt_elapsed_time(&_instance[i].timestamp_last) > status_timeout)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
			_instance[i].timeout = true;
		}

		// if the gyro used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_gyro_fault_detected && (faulty_gyro_id != 0) && (_instance[i].gyro_device_id == faulty_gyro_id)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
		}

		// if the accelerometer used by the EKF is faulty, declare the EKF unhealthy without delay
		if (_accel_fault_detected && (faulty_accel_id != 0) && (_instance[i].accel_device_id == faulty_accel_id)) {
			_instance[i].healthy.set_state_and_update(false, hrt_absolute_time());
		}

		if (prev_healthy != _instance[i].healthy.get_state()) {
			updated = true;
			_selector_status_publish = true;

			if (!prev_healthy) {
				_instance[i].healthy_count++;
			}
		}
	}

	// update relative test ratios if primary has updated
	if (primary_updated) {
		for (uint8_t i = 0; i < _available_instances; i++) {
			if (i != _selected_instance) {

				const float error_delta = _instance[i].combined_test_ratio - _instance[_selected_instance].combined_test_ratio;

				// reduce error only if its better than the primary instance by at least EKF2_SEL_ERR_RED to prevent unnecessary selection changes
				const float threshold = _gyro_fault_detected ? 0.0f : fmaxf(_param_ekf2_sel_err_red.get(), 0.05f);

				if (error_delta > 0 || error_delta < -threshold) {
					_instance[i].relative_test_ratio += error_delta;
					_instance[i].relative_test_ratio = constrain(_instance[i].relative_test_ratio, -_rel_err_score_lim, _rel_err_score_lim);

					if ((error_delta < -threshold) && (_instance[i].relative_test_ratio < 1.f)) {
						// increase status publication rate if there's movement towards a potential instance change
						_selector_status_publish = true;
					}
				}
			}
		}
	}

	return (primary_updated || updated);
}

void EKF2Selector::PublishVehicleAttitude()
{
	// selected estimator_attitude -> vehicle_attitude
	vehicle_attitude_s attitude;

	if (_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_attitude_sub.get_instance() != _attitude_instance_prev) {
			_attitude_instance_prev = _instance[_selected_instance].estimator_attitude_sub.get_instance();
			instance_change = true;
		}

		if (_attitude_last.timestamp != 0) {
			if (!instance_change && (attitude.quat_reset_counter == _attitude_last.quat_reset_counter + 1)) {
				// propogate deltas from estimator data while maintaining the overall reset counts
				++_quat_reset_counter;
				_delta_q_reset = Quatf{attitude.delta_q_reset};

			} else if (instance_change || (attitude.quat_reset_counter != _attitude_last.quat_reset_counter)) {
				// on reset compute deltas from last published data
				++_quat_reset_counter;
				_delta_q_reset = (Quatf(attitude.q) * Quatf(_attitude_last.q).inversed()).normalized();
			}

		} else {
			_quat_reset_counter = attitude.quat_reset_counter;
			_delta_q_reset = Quatf{attitude.delta_q_reset};
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's attitude for system (vehicle_attitude) if it's stale
		if ((attitude.timestamp_sample <= _attitude_last.timestamp_sample)
		    || (hrt_elapsed_time(&attitude.timestamp) > 10_ms)) {

			publish = false;
		}

		// save last primary estimator_attitude as published with original resets
		_attitude_last = attitude;

		if (publish) {
			// republish with total reset count and current timestamp
			attitude.quat_reset_counter = _quat_reset_counter;
			_delta_q_reset.copyTo(attitude.delta_q_reset);

			attitude.timestamp = hrt_absolute_time();
			_vehicle_attitude_pub.publish(attitude);
		}
	}
}

void EKF2Selector::PublishVehicleLocalPosition()
{
	// selected estimator_local_position -> vehicle_local_position
	vehicle_local_position_s local_position;

	if (_instance[_selected_instance].estimator_local_position_sub.update(&local_position)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_local_position_sub.get_instance() != _local_position_instance_prev) {
			_local_position_instance_prev = _instance[_selected_instance].estimator_local_position_sub.get_instance();
			instance_change = true;
		}

		if (_local_position_last.timestamp != 0) {
			// XY reset
			if (!instance_change && (local_position.xy_reset_counter == _local_position_last.xy_reset_counter + 1)) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.delta_xy};

			} else if (instance_change || (local_position.xy_reset_counter != _local_position_last.xy_reset_counter)) {
				++_xy_reset_counter;
				_delta_xy_reset = Vector2f{local_position.x, local_position.y} - Vector2f{_local_position_last.x, _local_position_last.y};
			}

			// Z reset
			if (!instance_change && (local_position.z_reset_counter == _local_position_last.z_reset_counter + 1)) {
				++_z_reset_counter;
				_delta_z_reset = local_position.delta_z;

			} else if (instance_change || (local_position.z_reset_counter != _local_position_last.z_reset_counter)) {
				++_z_reset_counter;
				_delta_z_reset = local_position.z - _local_position_last.z;
			}

			// VXY reset
			if (!instance_change && (local_position.vxy_reset_counter == _local_position_last.vxy_reset_counter + 1)) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.delta_vxy};

			} else if (instance_change || (local_position.vxy_reset_counter != _local_position_last.vxy_reset_counter)) {
				++_vxy_reset_counter;
				_delta_vxy_reset = Vector2f{local_position.vx, local_position.vy} - Vector2f{_local_position_last.vx, _local_position_last.vy};
			}

			// VZ reset
			if (!instance_change && (local_position.vz_reset_counter == _local_position_last.vz_reset_counter + 1)) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.delta_vz;

			} else if (instance_change || (local_position.vz_reset_counter != _local_position_last.vz_reset_counter)) {
				++_vz_reset_counter;
				_delta_vz_reset = local_position.vz - _local_position_last.vz;
			}

			// heading reset
			if (!instance_change && (local_position.heading_reset_counter == _local_position_last.heading_reset_counter + 1)) {
				++_heading_reset_counter;
				_delta_heading_reset = local_position.delta_heading;

			} else if (instance_change || (local_position.heading_reset_counter != _local_position_last.heading_reset_counter)) {
				++_heading_reset_counter;
				_delta_heading_reset = matrix::wrap_pi(local_position.heading - _local_position_last.heading);
			}

			// HAGL (dist_bottom) reset
			if (!instance_change
			    && (local_position.dist_bottom_reset_counter == _local_position_last.dist_bottom_reset_counter + 1)) {
				++_hagl_reset_counter;
				_delta_hagl_reset = local_position.delta_dist_bottom;

			} else if (instance_change
				   || (local_position.dist_bottom_reset_counter != _local_position_last.dist_bottom_reset_counter)) {
				++_hagl_reset_counter;
				_delta_hagl_reset = local_position.dist_bottom - _local_position_last.dist_bottom;
			}

		} else {
			_xy_reset_counter = local_position.xy_reset_counter;
			_z_reset_counter = local_position.z_reset_counter;
			_vxy_reset_counter = local_position.vxy_reset_counter;
			_vz_reset_counter = local_position.vz_reset_counter;
			_heading_reset_counter = local_position.heading_reset_counter;
			_hagl_reset_counter = local_position.dist_bottom_reset_counter;

			_delta_xy_reset = Vector2f{local_position.delta_xy};
			_delta_z_reset = local_position.delta_z;
			_delta_vxy_reset = Vector2f{local_position.delta_vxy};
			_delta_vz_reset = local_position.delta_vz;
			_delta_heading_reset = local_position.delta_heading;
			_delta_hagl_reset = local_position.delta_dist_bottom;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's local position for system (vehicle_local_position) if it's stale
		if ((local_position.timestamp_sample <= _local_position_last.timestamp_sample)
		    || (hrt_elapsed_time(&local_position.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_local_position as published with original resets
		_local_position_last = local_position;

		if (publish) {
			// republish with total reset count and current timestamp
			local_position.xy_reset_counter = _xy_reset_counter;
			local_position.z_reset_counter = _z_reset_counter;
			local_position.vxy_reset_counter = _vxy_reset_counter;
			local_position.vz_reset_counter = _vz_reset_counter;
			local_position.heading_reset_counter = _heading_reset_counter;
			local_position.dist_bottom_reset_counter = _hagl_reset_counter;

			_delta_xy_reset.copyTo(local_position.delta_xy);
			local_position.delta_z = _delta_z_reset;
			_delta_vxy_reset.copyTo(local_position.delta_vxy);
			local_position.delta_vz = _delta_vz_reset;
			local_position.delta_heading = _delta_heading_reset;
			local_position.delta_dist_bottom = _delta_hagl_reset;

			local_position.timestamp = hrt_absolute_time();
			_vehicle_local_position_pub.publish(local_position);
		}
	}
}

void EKF2Selector::PublishVehicleOdometry()
{
	// selected estimator_odometry -> vehicle_odometry
	vehicle_odometry_s odometry;

	if (_instance[_selected_instance].estimator_odometry_sub.update(&odometry)) {

		bool instance_change = false;

		if (_instance[_selected_instance].estimator_odometry_sub.get_instance() != _odometry_instance_prev) {
			_odometry_instance_prev = _instance[_selected_instance].estimator_odometry_sub.get_instance();
			instance_change = true;
		}

		if (_odometry_last.timestamp != 0) {
			// reset
			if (instance_change || (odometry.reset_counter != _odometry_last.reset_counter)) {
				++_odometry_reset_counter;
			}

		} else {
			_odometry_reset_counter = odometry.reset_counter;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's odometry for system (vehicle_odometry) if it's stale
		if ((odometry.timestamp_sample <= _odometry_last.timestamp_sample)
		    || (hrt_elapsed_time(&odometry.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_odometry as published with original resets
		_odometry_last = odometry;

		if (publish) {
			// republish with total reset count and current timestamp
			odometry.reset_counter = _odometry_reset_counter;

			odometry.timestamp = hrt_absolute_time();
			_vehicle_odometry_pub.publish(odometry);
		}
	}
}

void EKF2Selector::PublishVehicleGlobalPosition()
{
	// selected estimator_global_position -> vehicle_global_position
	vehicle_global_position_s global_position;

	if (_instance[_selected_instance].estimator_global_position_sub.update(&global_position)) {
		bool instance_change = false;

		if (_instance[_selected_instance].estimator_global_position_sub.get_instance() != _global_position_instance_prev) {
			_global_position_instance_prev = _instance[_selected_instance].estimator_global_position_sub.get_instance();
			instance_change = true;
		}

		if (_global_position_last.timestamp != 0) {
			// lat/lon reset
			if (!instance_change && (global_position.lat_lon_reset_counter == _global_position_last.lat_lon_reset_counter + 1)) {
				++_lat_lon_reset_counter;

				// TODO: delta latitude/longitude
				_delta_lat_reset = global_position.lat - _global_position_last.lat;
				_delta_lon_reset = global_position.lon - _global_position_last.lon;

			} else if (instance_change || (global_position.lat_lon_reset_counter != _global_position_last.lat_lon_reset_counter)) {
				++_lat_lon_reset_counter;

				_delta_lat_reset = global_position.lat - _global_position_last.lat;
				_delta_lon_reset = global_position.lon - _global_position_last.lon;
			}

			// alt reset
			if (!instance_change && (global_position.alt_reset_counter == _global_position_last.alt_reset_counter + 1)) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt;

			} else if (instance_change || (global_position.alt_reset_counter != _global_position_last.alt_reset_counter)) {
				++_alt_reset_counter;
				_delta_alt_reset = global_position.delta_alt - _global_position_last.delta_alt;
			}

			// terrain reset
			if (!instance_change && (global_position.terrain_reset_counter == _global_position_last.terrain_reset_counter + 1)) {
				++_terrain_reset_counter;
				_delta_terrain_reset = global_position.delta_terrain;

			} else if (instance_change || (global_position.terrain_reset_counter != _global_position_last.terrain_reset_counter)) {
				++_terrain_reset_counter;
				_delta_terrain_reset = global_position.delta_terrain - _global_position_last.delta_terrain;
			}

		} else {
			_lat_lon_reset_counter = global_position.lat_lon_reset_counter;
			_alt_reset_counter = global_position.alt_reset_counter;

			_delta_alt_reset = global_position.delta_alt;
		}

		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's global position for system (vehicle_global_position) if it's stale
		if ((global_position.timestamp_sample <= _global_position_last.timestamp_sample)
		    || (hrt_elapsed_time(&global_position.timestamp) > 20_ms)) {

			publish = false;
		}

		// save last primary estimator_global_position as published with original resets
		_global_position_last = global_position;

		if (publish) {
			// republish with total reset count and current timestamp
			global_position.lat_lon_reset_counter = _lat_lon_reset_counter;
			global_position.alt_reset_counter = _alt_reset_counter;
			global_position.delta_alt = _delta_alt_reset;

			global_position.timestamp = hrt_absolute_time();
			_vehicle_global_position_pub.publish(global_position);
		}
	}
}

void EKF2Selector::PublishWindEstimate()
{
	// selected estimator_wind -> wind
	wind_s wind;

	if (_instance[_selected_instance].estimator_wind_sub.update(&wind)) {
		bool publish = true;

		// ensure monotonically increasing timestamp_sample through reset, don't publish
		//  estimator's wind for system (wind) if it's stale
		if ((wind.timestamp_sample <= _wind_last.timestamp_sample)
		    || (hrt_elapsed_time(&wind.timestamp) > 100_ms)) {

			publish = false;
		}

		// save last primary wind
		_wind_last = wind;

		// publish estimator's wind for system unless it's stale
		if (publish) {
			// republish with current timestamp
			wind.timestamp = hrt_absolute_time();
			_wind_pub.publish(wind);
		}
	}
}

void EKF2Selector::Run()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// update combined test ratio for all estimators
	const bool updated = UpdateErrorScores();

	// if no valid instance then force select first instance with valid IMU
	if (_selected_instance == INVALID_INSTANCE) {
		// FORMIC_EST_MODE 1: EKF2 flies first
		const uint8_t instances_to_try = _formic_mode ? FORMIC_EKF_INSTANCE + 1 : EKF2_MAX_INSTANCES;

		for (uint8_t i = 0; i < instances_to_try; i++) {
			if ((_instance[i].accel_device_id != 0)
			    && (_instance[i].gyro_device_id != 0)) {

				if (SelectInstance(i)) {
					break;
				}
			}
		}

		// if still invalid return early and check again on next scheduled run
		if (_selected_instance == INVALID_INSTANCE) {
			ScheduleDelayed(100_ms);
			return;
		}
	}

	if (_formic_mode) {
		FormicRun();
		ScheduleDelayed(FILTER_UPDATE_PERIOD);
		return;
	}

	if (updated) {
		const uint8_t available_instances_prev = _available_instances;
		const uint8_t selected_instance_prev = _selected_instance;
		const uint32_t instance_changed_count_prev = _instance_changed_count;
		const hrt_abstime last_instance_change_prev = _last_instance_change;

		bool lower_error_available = false;
		float alternative_error = 0.f; // looking for instances that have error lower than the current primary
		float best_test_ratio = FLT_MAX;

		uint8_t best_ekf = _selected_instance;
		uint8_t best_ekf_alternate = INVALID_INSTANCE;
		uint8_t best_ekf_different_imu = INVALID_INSTANCE;

		// loop through all available instances to find if an alternative is available
		for (int i = 0; i < _available_instances; i++) {
			// Use an alternative instance if  -
			// (healthy and has updated recently)
			// AND
			// (has relative error less than selected instance and has not been the selected instance for at least 10 seconds
			// OR
			// selected instance has stopped updating
			if (_instance[i].healthy.get_state() && (i != _selected_instance)) {
				const float test_ratio = _instance[i].combined_test_ratio;
				const float relative_error = _instance[i].relative_test_ratio;

				if (relative_error < alternative_error) {
					best_ekf_alternate = i;
					alternative_error = relative_error;

					// relative error less than selected instance and has not been the selected instance for at least 10 seconds
					if ((relative_error <= -_rel_err_thresh) && hrt_elapsed_time(&_instance[i].time_last_selected) > 10_s) {
						lower_error_available = true;
					}
				}

				if ((test_ratio > 0) && (test_ratio < best_test_ratio)) {
					best_ekf = i;
					best_test_ratio = test_ratio;

					// also check next best available ekf using a different IMU
					if (_instance[i].accel_device_id != _instance[_selected_instance].accel_device_id) {
						best_ekf_different_imu = i;
					}
				}
			}
		}

		if (!_instance[_selected_instance].healthy.get_state()) {
			// prefer the best healthy instance using a different IMU
			if (!SelectInstance(best_ekf_different_imu)) {
				// otherwise switch to the healthy instance with best overall test ratio
				SelectInstance(best_ekf);
			}

		} else if (lower_error_available
			   && ((hrt_elapsed_time(&_last_instance_change) > 10_s)
			       || (_instance[_selected_instance].warning
				   && (hrt_elapsed_time(&_instance[_selected_instance].time_last_no_warning) > 1_s)))) {

			// if this instance has a significantly lower relative error to the active primary, we consider it as a
			// better instance and would like to switch to it even if the current primary is healthy
			SelectInstance(best_ekf_alternate);

		} else if (_request_instance.load() != INVALID_INSTANCE) {

			const uint8_t new_instance = _request_instance.load();

			// attempt to switch to user manually selected instance
			if (!SelectInstance(new_instance)) {
				PX4_ERR("unable to switch to user selected instance %d", new_instance);
			}

			// reset
			_request_instance.store(INVALID_INSTANCE);
		}

		// publish selector status at ~1 Hz or immediately on any change
		if (_selector_status_publish || (hrt_elapsed_time(&_last_status_publish) > 1_s)
		    || (available_instances_prev != _available_instances)
		    || (selected_instance_prev != _selected_instance)
		    || (instance_changed_count_prev != _instance_changed_count)
		    || (last_instance_change_prev != _last_instance_change)
		    || _accel_fault_detected || _gyro_fault_detected) {

			PublishEstimatorSelectorStatus();
			_selector_status_publish = false;
		}
	}

	// republish selected estimator data for system
	PublishVehicleAttitude();
	PublishVehicleLocalPosition();
	PublishVehicleGlobalPosition();
	PublishVehicleOdometry();
	PublishWindEstimate();

	// re-schedule as backup timeout
	ScheduleDelayed(FILTER_UPDATE_PERIOD);
}

void EKF2Selector::PublishEstimatorSelectorStatus()
{
	estimator_selector_status_s selector_status{};
	selector_status.primary_instance = _selected_instance;
	selector_status.instances_available = _available_instances;
	selector_status.instance_changed_count = _instance_changed_count;
	selector_status.last_instance_change = _last_instance_change;
	selector_status.accel_device_id = _instance[_selected_instance].accel_device_id;
	selector_status.baro_device_id = _instance[_selected_instance].baro_device_id;
	selector_status.gyro_device_id = _instance[_selected_instance].gyro_device_id;
	selector_status.mag_device_id = _instance[_selected_instance].mag_device_id;
	selector_status.gyro_fault_detected = _gyro_fault_detected;
	selector_status.accel_fault_detected = _accel_fault_detected;

	for (int i = 0; i < EKF2_MAX_INSTANCES; i++) {
		selector_status.combined_test_ratio[i] = _instance[i].combined_test_ratio;
		selector_status.relative_test_ratio[i] = _instance[i].relative_test_ratio;
		selector_status.healthy[i] = _instance[i].healthy.get_state();
	}

	for (int i = 0; i < IMU_STATUS_SIZE; i++) {
		selector_status.accumulated_gyro_error[i] = _accumulated_gyro_error[i];
		selector_status.accumulated_accel_error[i] = _accumulated_accel_error[i];
	}

	if (_formic_mode) {
		selector_status.formic_role_active = true;
		selector_status.role_primary = _formic_role;

		for (int i = 0; i < 2; i++) {
			selector_status.anchor_yaw[i] = _formic_anchor[i].psi;
			selector_status.anchor_valid[i] = _formic_anchor_valid[i];

			for (int j = 0; j < 3; j++) {
				selector_status.anchor_shift[3 * i + j] = _formic_anchor[i].t(j);
			}
		}

		selector_status.vio_segment = _formic_anchor_segment;
		selector_status.ramp_active = _formic_ramp_active;
		selector_status.handover_count = _formic_handover_count;
		selector_status.ekf_vel_seed_pending = _formic_seed_pending;
	}

	selector_status.timestamp = hrt_absolute_time();
	_estimator_selector_status_pub.publish(selector_status);
	_last_status_publish = selector_status.timestamp;
}

void EKF2Selector::PrintStatus()
{
	PX4_INFO("available instances: %" PRIu8, _available_instances);

	if (_selected_instance == INVALID_INSTANCE) {
		PX4_WARN("selected instance: None");
	}

	for (int i = 0; i < _available_instances; i++) {
		const EstimatorInstance &inst = _instance[i];

		PX4_INFO("%" PRIu8 ": ACC: %" PRIu32 ", GYRO: %" PRIu32 ", MAG: %" PRIu32 ", %s, test ratio: %.7f (%.5f) %s",
			 inst.instance, inst.accel_device_id, inst.gyro_device_id, inst.mag_device_id,
			 inst.healthy.get_state() ? "healthy" : "unhealthy",
			 (double)inst.combined_test_ratio, (double)inst.relative_test_ratio,
			 (_selected_instance == i) ? "*" : "");
	}

	if (_formic_mode) {
		PX4_INFO("FORMIC_EST_MODE 1: role %" PRIu8 "%s, %" PRIu32 " handovers, tilt ramp %s, velocity seed %s",
			 _formic_role, (_formic_override != INVALID_INSTANCE) ? " (select_instance)" : "", _formic_handover_count,
			 _formic_ramp_active ? "on" : "off", _formic_seed_pending ? "pending" : "idle");

		for (int i = 0; i < 2; i++) {
			PX4_INFO("anchor %d: %s yaw %.2f deg, shift %.2f %.2f %.2f m%s", i, _formic_anchor_valid[i] ? "" : "(none)",
				 (double)math::degrees(_formic_anchor[i].psi), (double)_formic_anchor[i].t(0), (double)_formic_anchor[i].t(1),
				 (double)_formic_anchor[i].t(2), (i == FORMIC_VIO_INSTANCE) ? "" : "");
		}

		PX4_INFO("instance %d anchor belongs to VIO segment %" PRIu32, FORMIC_VIO_INSTANCE, _formic_anchor_segment);
	}
}

void EKF2Selector::FormicRun()
{
	const hrt_abstime now = hrt_absolute_time();

	formic_pos_req_s pos_req{};
	_formic_pos_req_sub.copy(&pos_req);

	formic_vio_est_status_s vio{};
	const bool vio_fresh = _formic_vio_est_status_sub.copy(&vio) && (now < vio.timestamp + 200_ms);

	// ekf2 select_instance: a bench override of the role, until changed (any other value clears it)
	const uint8_t request = _request_instance.load();

	if (request != INVALID_INSTANCE) {
		_formic_override = (request <= FORMIC_VIO_INSTANCE) ? request : INVALID_INSTANCE;
		_request_instance.store(INVALID_INSTANCE);
		PX4_INFO("role override: %s", (_formic_override == INVALID_INSTANCE) ? "cleared" :
			 (_formic_override == FORMIC_VIO_INSTANCE) ? "VIO" : "EKF2");
	}

	const bool vio_available = _available_instances > FORMIC_VIO_INSTANCE;
	const bool vio_can_enter = vio_available && vio_fresh && vio.pos_offerable
				   && _instance[FORMIC_VIO_INSTANCE].healthy.get_state();
	const bool vio_can_stay = vio_available && vio_fresh && !vio.gap_exceeded
				  && !_instance[FORMIC_VIO_INSTANCE].timeout;
	const bool want_vio = (_formic_override != INVALID_INSTANCE) ? (_formic_override == FORMIC_VIO_INSTANCE)
			      : pos_req.use_vision_now;

	uint8_t role = FORMIC_EKF_INSTANCE;

	if (_selected_instance == FORMIC_VIO_INSTANCE) {
		role = (want_vio && vio_can_stay) ? FORMIC_VIO_INSTANCE : FORMIC_EKF_INSTANCE;

		// backstop: the VIO instance cannot carry on and commander has not left Position mode yet
		if (want_vio && !vio_can_stay) {
			if (!_formic_backstop_warned) {
				PX4_WARN("VIO instance failed (gap over FORMIC_VIO_GAP): switching to EKF2");
				_formic_backstop_warned = true;
			}

			_formic_override = INVALID_INSTANCE;
		}

	} else {
		role = (want_vio && vio_can_enter) ? FORMIC_VIO_INSTANCE : FORMIC_EKF_INSTANCE;
	}

	if (role != _formic_role) {
		_selector_status_publish = true;
	}

	_formic_role = role;

	if (role == _selected_instance) {
		_formic_seed_pending = false;

		if (role == FORMIC_VIO_INSTANCE) {
			_formic_backstop_warned = false;
		}

	} else if (role == FORMIC_EKF_INSTANCE) {
		// a handover because the VIO failed steps the tilt: ramping away from bad data only makes it last longer
		const bool requested = vio_fresh && vio.healthy;
		const bool seed = (_param_formic_ho_vseed.get() != 0) && vio_fresh && vio.vel_trusted
				  && !_instance[FORMIC_EKF_INSTANCE].timeout;
		bool switch_now = true;

		if (seed) {
			if (!_formic_seed_pending) {
				// switch once EKF2 has published the VIO's horizontal velocity, so its velocity does not step
				_formic_seed_pending = FormicRequestVelocitySeed();
				switch_now = !_formic_seed_pending;

			} else {
				vehicle_local_position_s lpos0;

				if (_instance[FORMIC_EKF_INSTANCE].estimator_local_position_sub.copy(&lpos0)
				    && (lpos0.vxy_reset_counter != _formic_seed_vxy_counter)) {
					switch_now = true;

				} else if (now > _formic_seed_deadline) {
					PX4_WARN("EKF2 did not take the VIO velocity in time: switching without it");
					switch_now = true;

				} else {
					switch_now = false;
				}
			}
		}

		if (switch_now) {
			_formic_seed_pending = false;
			FormicHandover(FORMIC_EKF_INSTANCE, requested, vio.segment);
		}

	} else {
		_formic_seed_pending = false;
		FormicHandover(FORMIC_VIO_INSTANCE, true, vio.segment);
	}

	// the selector status carries the anchors: logs and the Jetson convert between the frames with it
	if (_selector_status_publish || (now > _last_status_publish + 100_ms) || _accel_fault_detected
	    || _gyro_fault_detected) {
		PublishEstimatorSelectorStatus();
		_selector_status_publish = false;
	}

	FormicPublishVehicleAttitude();
	FormicPublishVehicleLocalPosition();
	PublishVehicleGlobalPosition();
	FormicPublishVehicleOdometry();
	PublishWindEstimate();
}

bool EKF2Selector::FormicRequestVelocitySeed()
{
	vehicle_attitude_s att0;
	vehicle_local_position_s lpos0;

	if (!_instance[FORMIC_EKF_INSTANCE].estimator_attitude_sub.copy(&att0)
	    || !_instance[FORMIC_EKF_INSTANCE].estimator_local_position_sub.copy(&lpos0)) {
		return false;
	}

	formic_frames::PoseSample pub_att;
	formic_frames::PoseSample pub_lpos;

	if (!_formic_att_history.sampleAt(att0.timestamp_sample, pub_att)
	    || !_formic_lpos_history.sampleAt(lpos0.timestamp_sample, pub_lpos)) {
		return false;
	}

	// EKF2 will be anchored onto the published heading: its frame differs from the published one by this yaw
	const formic_frames::Anchor anchor{matrix::wrap_pi(formic_frames::yaw_of(pub_att.q) - formic_frames::yaw_of(Quatf(att0.q))), {}};
	const Vector2f vel_ne = anchor.unapplyVector(Vector2f(pub_lpos.v(0), pub_lpos.v(1)));

	formic_ekf_vel_reset_s reset{};
	reset.instance = FORMIC_EKF_INSTANCE;
	vel_ne.copyTo(reset.vel_ne);
	reset.vel_var = math::max(_local_position_last.evh * _local_position_last.evh, 0.01f);
	reset.sequence = ++_formic_seed_sequence;
	reset.timestamp = hrt_absolute_time();
	_formic_ekf_vel_reset_pub.publish(reset);

	_formic_seed_vxy_counter = lpos0.vxy_reset_counter;
	_formic_seed_deadline = reset.timestamp + 100_ms;
	return true;
}

void EKF2Selector::FormicHandover(uint8_t new_instance, bool requested, uint32_t vio_segment)
{
	vehicle_attitude_s att;
	vehicle_local_position_s lpos;

	if (!_instance[new_instance].estimator_attitude_sub.copy(&att)
	    || !_instance[new_instance].estimator_local_position_sub.copy(&lpos)) {
		return;
	}

	const uint8_t old_instance = _selected_instance;
	const Quatf q_inst(att.q);
	const Vector3f p_inst(lpos.x, lpos.y, lpos.z);
	const Vector3f v_inst(lpos.vx, lpos.vy, lpos.vz);

	// the published pose at the new instance's sample times
	formic_frames::PoseSample pub_att;
	formic_frames::PoseSample pub_lpos;
	const bool have_pub = _formic_att_history.sampleAt(att.timestamp_sample, pub_att)
			      && _formic_lpos_history.sampleAt(lpos.timestamp_sample, pub_lpos);

	formic_frames::Anchor anchor = _formic_anchor[new_instance];
	bool snap_back = false;

	if (!have_pub) {
		if (!_formic_anchor_valid[new_instance]) {
			anchor = formic_frames::Anchor{};
		}

	} else if ((new_instance == FORMIC_VIO_INSTANCE) && _formic_anchor_valid[FORMIC_VIO_INSTANCE]
		   && (vio_segment == _formic_anchor_segment)) {
		// same VIO segment: back to its frame, stepping out EKF2's drift since the last switch
		snap_back = true;

	} else {
		// a new anchor: the published heading and position carry on
		anchor.psi = matrix::wrap_pi(formic_frames::yaw_of(pub_att.q) - formic_frames::yaw_of(q_inst));
		anchor.t = pub_lpos.p - matrix::Dcmf(anchor.rotation()) * p_inst;
	}

	const Quatf q_anchored = anchor.apply(q_inst);
	const Vector3f d_theta = have_pub ? formic_frames::tilt_offset(pub_att.q, q_anchored) : Vector3f{};
	const bool ramp = requested && have_pub && (_param_formic_ho_tau.get() > 0.f);
	const Quatf q_out = ramp ? formic_frames::apply_tilt(d_theta, 1.f, q_anchored) : q_anchored;
	const Vector3f p_out = anchor.applyPosition(p_inst);
	const Vector3f v_out = anchor.applyVector(v_inst);

	if (!SelectInstance(new_instance)) {
		return;
	}

	_formic_anchor[new_instance] = anchor;
	_formic_anchor_valid[new_instance] = true;

	if (new_instance == FORMIC_VIO_INSTANCE) {
		_formic_anchor_segment = vio_segment;
	}

	formic_frames::Step step{};

	if (have_pub) {
		step = formic_frames::step_between(pub_att.q, pub_lpos.p, pub_lpos.v, q_out, p_out, v_out);

		// the velocity always steps: the flight tasks and mc_pos_control restart from the new one
		_delta_vxy_reset = Vector2f(step.delta_v(0), step.delta_v(1));
		++_vxy_reset_counter;
		_delta_vz_reset = step.delta_v(2);
		++_vz_reset_counter;

		if (snap_back) {
			_delta_xy_reset = Vector2f(step.delta_p(0), step.delta_p(1));
			++_xy_reset_counter;
			_delta_z_reset = step.delta_p(2);
			++_z_reset_counter;
			_delta_heading_reset = step.delta_heading;
			++_heading_reset_counter;
		}

		if (snap_back || !ramp) {
			_delta_q_reset = step.delta_q;
			++_quat_reset_counter;
		}

		if (_local_position_last.dist_bottom_valid && lpos.dist_bottom_valid
		    && fabsf(lpos.dist_bottom - _local_position_last.dist_bottom) > 1e-3f) {
			_delta_hagl_reset = lpos.dist_bottom - _local_position_last.dist_bottom;
			++_hagl_reset_counter;
		}

		++_odometry_reset_counter;
	}

	_formic_ramp_active = ramp && (d_theta.norm() > 1e-5f);
	_formic_ramp_dtheta = d_theta;
	_formic_ramp_t0 = att.timestamp_sample;

	// take the new instance's own reset counters as they are now
	_formic_att_baseline = false;
	_formic_lpos_baseline = false;
	_formic_odom_baseline = false;

	_formic_handover_count++;
	_selector_status_publish = true;

	PX4_INFO("handover %" PRIu8 " -> %" PRIu8 " (%s): velocity step %.2f %.2f %.2f m/s, tilt %.2f deg %s%s",
		 old_instance, new_instance, requested ? "requested" : "VIO failed",
		 (double)step.delta_v(0), (double)step.delta_v(1), (double)step.delta_v(2),
		 (double)math::degrees(d_theta.norm()), ramp ? "ramped" : "stepped",
		 snap_back ? ", back to the VIO segment frame" : "");

	if (snap_back) {
		PX4_INFO("snap back: %.2f %.2f %.2f m, heading %.1f deg", (double)step.delta_p(0), (double)step.delta_p(1),
			 (double)step.delta_p(2), (double)math::degrees(step.delta_heading));
	}
}

Quatf EKF2Selector::FormicAttitudeOut(const Quatf &q, uint64_t timestamp_sample)
{
	const Quatf q_anchored = _formic_anchor[_selected_instance].apply(q);

	if (!_formic_ramp_active) {
		return q_anchored;
	}

	const float dt = (timestamp_sample > _formic_ramp_t0) ? (timestamp_sample - _formic_ramp_t0) * 1e-6f : 0.f;
	const float scale = formic_frames::ramp_scale(dt, _param_formic_ho_tau.get());

	if (scale < 1e-3f) {
		_formic_ramp_active = false;
		_selector_status_publish = true;
		return q_anchored;
	}

	return formic_frames::apply_tilt(_formic_ramp_dtheta, scale, q_anchored);
}

void EKF2Selector::FormicPublishVehicleAttitude()
{
	vehicle_attitude_s attitude;

	if (!_instance[_selected_instance].estimator_attitude_sub.update(&attitude)) {
		return;
	}

	const formic_frames::Anchor &anchor = _formic_anchor[_selected_instance];

	// the instance's own resets, turned into the published frame
	if (!_formic_att_baseline) {
		_formic_inst_quat_counter = attitude.quat_reset_counter;
		_formic_att_baseline = true;

	} else if (attitude.quat_reset_counter != _formic_inst_quat_counter) {
		_formic_inst_quat_counter = attitude.quat_reset_counter;
		++_quat_reset_counter;
		_delta_q_reset = anchor.applyDeltaQ(Quatf(attitude.delta_q_reset));
	}

	const Quatf q_out = FormicAttitudeOut(Quatf(attitude.q), attitude.timestamp_sample);

	// ensure monotonically increasing timestamp_sample through a handover, and don't publish a stale estimate
	if ((attitude.timestamp_sample <= _attitude_last.timestamp_sample) || (hrt_elapsed_time(&attitude.timestamp) > 10_ms)) {
		return;
	}

	q_out.copyTo(attitude.q);
	attitude.quat_reset_counter = _quat_reset_counter;
	_delta_q_reset.copyTo(attitude.delta_q_reset);
	attitude.timestamp = hrt_absolute_time();
	_vehicle_attitude_pub.publish(attitude);
	_attitude_last = attitude;

	formic_frames::PoseSample s;
	s.time_us = attitude.timestamp_sample;
	s.q = q_out;
	_formic_att_history.push(s);
}

void EKF2Selector::FormicPublishVehicleLocalPosition()
{
	vehicle_local_position_s lpos;

	if (!_instance[_selected_instance].estimator_local_position_sub.update(&lpos)) {
		return;
	}

	const formic_frames::Anchor &anchor = _formic_anchor[_selected_instance];

	// the instance's own resets, turned into the published frame
	if (!_formic_lpos_baseline) {
		_formic_inst_xy_counter = lpos.xy_reset_counter;
		_formic_inst_z_counter = lpos.z_reset_counter;
		_formic_inst_vxy_counter = lpos.vxy_reset_counter;
		_formic_inst_vz_counter = lpos.vz_reset_counter;
		_formic_inst_heading_counter = lpos.heading_reset_counter;
		_formic_inst_hagl_counter = lpos.dist_bottom_reset_counter;
		_formic_lpos_baseline = true;

	} else {
		if (lpos.xy_reset_counter != _formic_inst_xy_counter) {
			_formic_inst_xy_counter = lpos.xy_reset_counter;
			++_xy_reset_counter;
			_delta_xy_reset = anchor.applyVector(Vector2f(lpos.delta_xy));
		}

		if (lpos.z_reset_counter != _formic_inst_z_counter) {
			_formic_inst_z_counter = lpos.z_reset_counter;
			++_z_reset_counter;
			_delta_z_reset = lpos.delta_z;
		}

		if (lpos.vxy_reset_counter != _formic_inst_vxy_counter) {
			_formic_inst_vxy_counter = lpos.vxy_reset_counter;
			++_vxy_reset_counter;
			_delta_vxy_reset = anchor.applyVector(Vector2f(lpos.delta_vxy));
		}

		if (lpos.vz_reset_counter != _formic_inst_vz_counter) {
			_formic_inst_vz_counter = lpos.vz_reset_counter;
			++_vz_reset_counter;
			_delta_vz_reset = lpos.delta_vz;
		}

		if (lpos.heading_reset_counter != _formic_inst_heading_counter) {
			_formic_inst_heading_counter = lpos.heading_reset_counter;
			++_heading_reset_counter;
			_delta_heading_reset = lpos.delta_heading;
		}

		if (lpos.dist_bottom_reset_counter != _formic_inst_hagl_counter) {
			_formic_inst_hagl_counter = lpos.dist_bottom_reset_counter;
			++_hagl_reset_counter;
			_delta_hagl_reset = lpos.delta_dist_bottom;
		}
	}

	if ((lpos.timestamp_sample <= _local_position_last.timestamp_sample)
	    || (hrt_elapsed_time(&lpos.timestamp) > 20_ms)) {
		return;
	}

	const Vector3f p = anchor.applyPosition(Vector3f(lpos.x, lpos.y, lpos.z));
	const Vector3f v = anchor.applyVector(Vector3f(lpos.vx, lpos.vy, lpos.vz));
	const Vector3f a = anchor.applyVector(Vector3f(lpos.ax, lpos.ay, lpos.az));

	lpos.x = p(0);
	lpos.y = p(1);
	lpos.z = p(2);
	lpos.vx = v(0);
	lpos.vy = v(1);
	lpos.vz = v(2);
	lpos.ax = a(0);
	lpos.ay = a(1);
	lpos.az = a(2);
	lpos.heading = anchor.applyHeading(lpos.heading);
	lpos.unaided_heading = anchor.applyHeading(lpos.unaided_heading);

	lpos.xy_reset_counter = _xy_reset_counter;
	lpos.z_reset_counter = _z_reset_counter;
	lpos.vxy_reset_counter = _vxy_reset_counter;
	lpos.vz_reset_counter = _vz_reset_counter;
	lpos.heading_reset_counter = _heading_reset_counter;
	lpos.dist_bottom_reset_counter = _hagl_reset_counter;

	_delta_xy_reset.copyTo(lpos.delta_xy);
	lpos.delta_z = _delta_z_reset;
	_delta_vxy_reset.copyTo(lpos.delta_vxy);
	lpos.delta_vz = _delta_vz_reset;
	lpos.delta_heading = _delta_heading_reset;
	lpos.delta_dist_bottom = _delta_hagl_reset;

	lpos.timestamp = hrt_absolute_time();
	_vehicle_local_position_pub.publish(lpos);
	_local_position_last = lpos;

	formic_frames::PoseSample s;
	s.time_us = lpos.timestamp_sample;
	s.p = p;
	s.v = v;
	_formic_lpos_history.push(s);
}

void EKF2Selector::FormicPublishVehicleOdometry()
{
	vehicle_odometry_s odometry;

	if (!_instance[_selected_instance].estimator_odometry_sub.update(&odometry)) {
		return;
	}

	if (!_formic_odom_baseline) {
		_formic_inst_odom_counter = odometry.reset_counter;
		_formic_odom_baseline = true;

	} else if (odometry.reset_counter != _formic_inst_odom_counter) {
		_formic_inst_odom_counter = odometry.reset_counter;
		++_odometry_reset_counter;
	}

	if ((odometry.timestamp_sample <= _odometry_last.timestamp_sample)
	    || (hrt_elapsed_time(&odometry.timestamp) > 20_ms)) {
		return;
	}

	const formic_frames::Anchor &anchor = _formic_anchor[_selected_instance];

	if (odometry.pose_frame == vehicle_odometry_s::POSE_FRAME_NED) {
		anchor.applyPosition(Vector3f(odometry.position)).copyTo(odometry.position);
		FormicAttitudeOut(Quatf(odometry.q), odometry.timestamp_sample).copyTo(odometry.q);
	}

	if (odometry.velocity_frame == vehicle_odometry_s::VELOCITY_FRAME_NED) {
		anchor.applyVector(Vector3f(odometry.velocity)).copyTo(odometry.velocity);
	}

	odometry.reset_counter = _odometry_reset_counter;
	odometry.timestamp = hrt_absolute_time();
	_vehicle_odometry_pub.publish(odometry);
	_odometry_last = odometry;
}




// void EKF2Selector::update_heading_for_instance(uint8_t a, uint8_t b)
// {
// 	// Take the heading from instance a and apply it to instance b.
// 	if (a >= EKF2_MAX_INSTANCES || b >= EKF2_MAX_INSTANCES) {
// 		return;
// 	}
// 	vehicle_attitude_s att{};

// 	if (!_instance[a].estimator_attitude_sub.copy(&att) || att.timestamp == 0) {
// 		return;
// 	}
// 	const float yaw =  matrix::Eulerf(Quatf(att.q)).psi();
// 	vehicle_command_s cmd{};
// 	cmd.command          = vehicle_command_s::VEHICLE_CMD_EXTERNAL_ATTITUDE_ESTIMATE;
// 	cmd.param3           = math::degrees(yaw); // heading in degrees
// 	cmd.param7           = 5.f;                // accuracy in degrees
// 	cmd.target_component = b + 1;              // 1-based: targets only instance b
// 	cmd.timestamp        = hrt_absolute_time();
// 	_vehicle_command_pub.publish(cmd);
// }





