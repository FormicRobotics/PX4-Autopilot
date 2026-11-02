#include <gtest/gtest.h>

#include "control_state.hpp"

using formic_frames::control_state_from;

namespace
{
vehicle_local_position_s makeLocalPosition()
{
	vehicle_local_position_s lpos{};
	lpos.timestamp = 1'000'500;
	lpos.timestamp_sample = 1'000'000;
	lpos.x = 1.5f;
	lpos.y = -2.25f;
	lpos.z = -3.125f;
	lpos.vx = 0.5f;
	lpos.vy = -0.75f;
	lpos.vz = 0.125f;
	lpos.heading = 1.2f;
	lpos.xy_valid = true;
	lpos.z_valid = false;
	lpos.v_xy_valid = true;
	lpos.v_z_valid = false;
	lpos.eph = 0.3f;
	lpos.epv = 0.4f;
	lpos.evh = 0.05f;
	lpos.evv = 0.06f;
	lpos.xy_reset_counter = 3;
	lpos.z_reset_counter = 4;
	lpos.vxy_reset_counter = 5;
	lpos.vz_reset_counter = 6;
	lpos.heading_reset_counter = 7;
	return lpos;
}

// rolled, pitched and yawed: no component is 0
vehicle_attitude_s makeAttitude()
{
	vehicle_attitude_s att{};
	att.timestamp = 1'000'400;
	att.timestamp_sample = 999'000;
	att.q[0] = 0.9f;
	att.q[1] = 0.1f;
	att.q[2] = -0.2f;
	att.q[3] = 0.3714835f;
	att.quat_reset_counter = 8;
	return att;
}
}

// Every field equals the vehicle_local_position or vehicle_attitude it was taken from.
TEST(FormicControlState, CopiesEveryField)
{
	const vehicle_local_position_s lpos = makeLocalPosition();
	const vehicle_attitude_s att = makeAttitude();
	const formic_control_state_s s = control_state_from(lpos, att, 1, 9, true);

	EXPECT_EQ(s.timestamp, 0u);
	EXPECT_EQ(s.timestamp_sample, lpos.timestamp_sample);
	EXPECT_EQ(s.attitude_timestamp_sample, att.timestamp_sample);

	EXPECT_EQ(s.instance, formic_control_state_s::INSTANCE_VIO);
	EXPECT_EQ(s.handover_count, 9u);
	EXPECT_TRUE(s.tilt_ramp_active);

	EXPECT_EQ(s.position[0], lpos.x);
	EXPECT_EQ(s.position[1], lpos.y);
	EXPECT_EQ(s.position[2], lpos.z);
	EXPECT_EQ(s.velocity[0], lpos.vx);
	EXPECT_EQ(s.velocity[1], lpos.vy);
	EXPECT_EQ(s.velocity[2], lpos.vz);
	EXPECT_EQ(s.heading, lpos.heading);

	for (int i = 0; i < 4; i++) {
		EXPECT_EQ(s.q[i], att.q[i]);
	}

	EXPECT_EQ(s.xy_valid, lpos.xy_valid);
	EXPECT_EQ(s.z_valid, lpos.z_valid);
	EXPECT_EQ(s.v_xy_valid, lpos.v_xy_valid);
	EXPECT_EQ(s.v_z_valid, lpos.v_z_valid);
	EXPECT_EQ(s.eph, lpos.eph);
	EXPECT_EQ(s.epv, lpos.epv);
	EXPECT_EQ(s.evh, lpos.evh);
	EXPECT_EQ(s.evv, lpos.evv);

	EXPECT_EQ(s.xy_reset_counter, lpos.xy_reset_counter);
	EXPECT_EQ(s.z_reset_counter, lpos.z_reset_counter);
	EXPECT_EQ(s.vxy_reset_counter, lpos.vxy_reset_counter);
	EXPECT_EQ(s.vz_reset_counter, lpos.vz_reset_counter);
	EXPECT_EQ(s.heading_reset_counter, lpos.heading_reset_counter);
	EXPECT_EQ(s.quat_reset_counter, att.quat_reset_counter);
}

// The EKF2 instance with no handover yet and no ramp.
TEST(FormicControlState, EkfInstance)
{
	const formic_control_state_s s = control_state_from(makeLocalPosition(), makeAttitude(), 0, 0, false);
	EXPECT_EQ(s.instance, formic_control_state_s::INSTANCE_EKF2);
	EXPECT_EQ(s.handover_count, 0u);
	EXPECT_FALSE(s.tilt_ramp_active);
}
