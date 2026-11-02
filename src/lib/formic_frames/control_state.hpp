#pragma once

#include <uORB/topics/formic_control_state.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

namespace formic_frames
{

/**
 * The state the controllers use, from the vehicle_local_position and vehicle_attitude just published.
 * Every value is copied as published; `timestamp` is left for the caller.
 * @param instance        estimator instance the two structs came from
 * @param handover_count  handovers between the instances so far
 * @param ramp_active     the tilt ramp after a handover is running
 */
inline formic_control_state_s control_state_from(const vehicle_local_position_s &lpos, const vehicle_attitude_s &att,
		uint8_t instance, uint32_t handover_count, bool ramp_active)
{
	formic_control_state_s s{};
	s.timestamp_sample = lpos.timestamp_sample;
	s.attitude_timestamp_sample = att.timestamp_sample;

	s.instance = instance;
	s.handover_count = handover_count;
	s.tilt_ramp_active = ramp_active;

	s.position[0] = lpos.x;
	s.position[1] = lpos.y;
	s.position[2] = lpos.z;
	s.velocity[0] = lpos.vx;
	s.velocity[1] = lpos.vy;
	s.velocity[2] = lpos.vz;
	s.heading = lpos.heading;

	for (int i = 0; i < 4; i++) {
		s.q[i] = att.q[i];
	}

	s.xy_valid = lpos.xy_valid;
	s.z_valid = lpos.z_valid;
	s.v_xy_valid = lpos.v_xy_valid;
	s.v_z_valid = lpos.v_z_valid;
	s.eph = lpos.eph;
	s.epv = lpos.epv;
	s.evh = lpos.evh;
	s.evv = lpos.evv;

	s.xy_reset_counter = lpos.xy_reset_counter;
	s.z_reset_counter = lpos.z_reset_counter;
	s.vxy_reset_counter = lpos.vxy_reset_counter;
	s.vz_reset_counter = lpos.vz_reset_counter;
	s.heading_reset_counter = lpos.heading_reset_counter;
	s.quat_reset_counter = att.quat_reset_counter;
	return s;
}

} // namespace formic_frames
