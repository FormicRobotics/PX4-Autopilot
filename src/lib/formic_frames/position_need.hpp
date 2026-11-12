/****************************************************************************
 * Whether a flight mode needs a position estimate, so the Formic VIO is requested for it.
 *
 * A mode needs position when its bit is set in any of the four position requirements of
 * failsafe_flags (local and global, strict and relaxed), or when it is Offboard and a fresh
 * offboard_control_mode asks for position or velocity control. Offboard's need is not in the
 * masks: it follows the setpoint type the companion sends.
 *
 * Shared by the commander (UserModeIntention) and the MSP OSD. No uORB calls: tested with gtest.
 ****************************************************************************/

#pragma once

#include <cstdint>

#include <uORB/topics/failsafe_flags.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_status.h>

namespace formic_frames
{

/// offboard_control_mode is fresh for this long when the caller has no COM_OF_LOSS_T (microseconds).
static constexpr uint64_t OFFBOARD_FRESH_DEFAULT_US = 1'000'000;

/// The mode's bit is set in one of the four position requirements.
inline bool mode_in_position_masks(const failsafe_flags_s &flags, uint8_t nav_state)
{
	if (nav_state >= 32) {
		return false;
	}

	const uint32_t bit = 1u << nav_state;
	return ((flags.mode_req_local_position | flags.mode_req_local_position_relaxed
		 | flags.mode_req_global_position | flags.mode_req_global_position_relaxed) & bit) != 0;
}

/// The Offboard setpoints ask for position or velocity control, and were published within `fresh_us`.
inline bool offboard_needs_position(const offboard_control_mode_s &offboard, uint64_t now_us,
				    uint64_t fresh_us = OFFBOARD_FRESH_DEFAULT_US)
{
	const bool fresh = (offboard.timestamp != 0) && (now_us < offboard.timestamp + fresh_us);
	return fresh && (offboard.position || offboard.velocity);
}

/**
 * True when `nav_state` needs a position estimate.
 * @param offboard  the newest offboard_control_mode; zero-initialised when there is none
 * @param fresh_us  how long offboard_control_mode counts after its timestamp (COM_OF_LOSS_T)
 */
inline bool mode_needs_position(const failsafe_flags_s &flags, uint8_t nav_state,
				const offboard_control_mode_s &offboard, uint64_t now_us,
				uint64_t fresh_us = OFFBOARD_FRESH_DEFAULT_US)
{
	if (mode_in_position_masks(flags, nav_state)) {
		return true;
	}

	return (nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD)
	       && offboard_needs_position(offboard, now_us, fresh_us);
}

} // namespace formic_frames
