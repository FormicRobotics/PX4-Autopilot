#include <gtest/gtest.h>

#include "position_need.hpp"

using formic_frames::mode_needs_position;

namespace
{
constexpr uint64_t kNowUs = 50'000'000;
constexpr uint64_t kFreshUs = 1'000'000;

constexpr uint8_t kPosctl = vehicle_status_s::NAVIGATION_STATE_POSCTL;
constexpr uint8_t kOffboard = vehicle_status_s::NAVIGATION_STATE_OFFBOARD;

constexpr uint32_t bit(uint8_t nav_state) { return 1u << nav_state; }

// a fresh offboard_control_mode with no setpoint type set
offboard_control_mode_s freshOffboard()
{
	offboard_control_mode_s offboard{};
	offboard.timestamp = kNowUs - 20'000;
	return offboard;
}
}

// A mode set in any one of the four position requirements needs position.
TEST(FormicPositionNeed, TrueForEachOfTheFourMasks)
{
	const offboard_control_mode_s none{};

	failsafe_flags_s local{};
	local.mode_req_local_position = bit(kPosctl);
	EXPECT_TRUE(mode_needs_position(local, kPosctl, none, kNowUs));

	failsafe_flags_s local_relaxed{};
	local_relaxed.mode_req_local_position_relaxed = bit(vehicle_status_s::NAVIGATION_STATE_AUTO_LAND);
	EXPECT_TRUE(mode_needs_position(local_relaxed, vehicle_status_s::NAVIGATION_STATE_AUTO_LAND, none, kNowUs));

	failsafe_flags_s global{};
	global.mode_req_global_position = bit(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	EXPECT_TRUE(mode_needs_position(global, vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION, none, kNowUs));

	failsafe_flags_s global_relaxed{};
	global_relaxed.mode_req_global_position_relaxed = bit(vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);
	EXPECT_TRUE(mode_needs_position(global_relaxed, vehicle_status_s::NAVIGATION_STATE_AUTO_RTL, none, kNowUs));

	// only the mode whose bit is set
	EXPECT_FALSE(mode_needs_position(local, vehicle_status_s::NAVIGATION_STATE_ALTCTL, none, kNowUs));
}

// Altitude, Stabilized, Acro and Manual are in no position mask.
TEST(FormicPositionNeed, FalseForModesWithoutPosition)
{
	failsafe_flags_s flags{};
	flags.mode_req_local_position = bit(kPosctl) | bit(vehicle_status_s::NAVIGATION_STATE_POSITION_SLOW);
	flags.mode_req_global_position = bit(vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION);
	const offboard_control_mode_s none{};

	EXPECT_FALSE(mode_needs_position(flags, vehicle_status_s::NAVIGATION_STATE_ALTCTL, none, kNowUs));
	EXPECT_FALSE(mode_needs_position(flags, vehicle_status_s::NAVIGATION_STATE_STAB, none, kNowUs));
	EXPECT_FALSE(mode_needs_position(flags, vehicle_status_s::NAVIGATION_STATE_ACRO, none, kNowUs));
	EXPECT_FALSE(mode_needs_position(flags, vehicle_status_s::NAVIGATION_STATE_MANUAL, none, kNowUs));
}

// Offboard needs position with position or velocity setpoints, and with none of the others.
TEST(FormicPositionNeed, OffboardFollowsTheSetpointType)
{
	const failsafe_flags_s flags{};

	offboard_control_mode_s position = freshOffboard();
	position.position = true;
	EXPECT_TRUE(mode_needs_position(flags, kOffboard, position, kNowUs, kFreshUs));

	offboard_control_mode_s velocity = freshOffboard();
	velocity.velocity = true;
	EXPECT_TRUE(mode_needs_position(flags, kOffboard, velocity, kNowUs, kFreshUs));

	offboard_control_mode_s attitude = freshOffboard();
	attitude.attitude = true;
	EXPECT_FALSE(mode_needs_position(flags, kOffboard, attitude, kNowUs, kFreshUs));

	offboard_control_mode_s body_rate = freshOffboard();
	body_rate.body_rate = true;
	EXPECT_FALSE(mode_needs_position(flags, kOffboard, body_rate, kNowUs, kFreshUs));

	offboard_control_mode_s thrust_and_torque = freshOffboard();
	thrust_and_torque.thrust_and_torque = true;
	EXPECT_FALSE(mode_needs_position(flags, kOffboard, thrust_and_torque, kNowUs, kFreshUs));

	offboard_control_mode_s direct_actuator = freshOffboard();
	direct_actuator.direct_actuator = true;
	EXPECT_FALSE(mode_needs_position(flags, kOffboard, direct_actuator, kNowUs, kFreshUs));
}

// A stale offboard_control_mode, or none at all, asks for nothing.
TEST(FormicPositionNeed, StaleOffboardIsFalse)
{
	const failsafe_flags_s flags{};

	offboard_control_mode_s stale{};
	stale.position = true;
	stale.timestamp = kNowUs - kFreshUs;
	EXPECT_FALSE(mode_needs_position(flags, kOffboard, stale, kNowUs, kFreshUs));

	stale.timestamp = kNowUs - kFreshUs + 1;
	EXPECT_TRUE(mode_needs_position(flags, kOffboard, stale, kNowUs, kFreshUs));

	offboard_control_mode_s never{};
	never.position = true;
	EXPECT_FALSE(mode_needs_position(flags, kOffboard, never, kNowUs, kFreshUs));
}

// offboard_control_mode counts for Offboard only.
TEST(FormicPositionNeed, OtherModesIgnoreOffboard)
{
	const failsafe_flags_s flags{};
	offboard_control_mode_s position = freshOffboard();
	position.position = true;

	EXPECT_FALSE(mode_needs_position(flags, vehicle_status_s::NAVIGATION_STATE_ALTCTL, position, kNowUs, kFreshUs));
	EXPECT_FALSE(mode_needs_position(flags, vehicle_status_s::NAVIGATION_STATE_STAB, position, kNowUs, kFreshUs));
	EXPECT_FALSE(mode_needs_position(flags, kPosctl, position, kNowUs, kFreshUs));
}

// A nav_state beyond the 32 bits of a mask is in none of them.
TEST(FormicPositionNeed, OutOfRangeModeIsFalse)
{
	failsafe_flags_s flags{};
	flags.mode_req_local_position = UINT32_MAX;
	EXPECT_FALSE(mode_needs_position(flags, 32, offboard_control_mode_s{}, kNowUs));
	EXPECT_FALSE(mode_needs_position(flags, UINT8_MAX, offboard_control_mode_s{}, kNowUs));
}
