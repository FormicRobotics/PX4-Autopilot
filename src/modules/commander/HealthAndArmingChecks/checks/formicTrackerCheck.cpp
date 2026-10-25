/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "formicTrackerCheck.hpp"

#include <lib/mathlib/mathlib.h>

using namespace time_literals;

void FormicTrackerChecks::checkAndReport(const Context &context, Report &reporter)
{
	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime loss_timeout = static_cast<hrt_abstime>(_param_formic_trk_loss_t.get() * 1_s);

	bool data_is_recent = false;
	formic_tracker_setpoint_s formic_tracker_setpoint;

	if (_formic_tracker_setpoint_sub.copy(&formic_tracker_setpoint)) {
		data_is_recent = now < formic_tracker_setpoint.timestamp + loss_timeout;
	}

	// Output watchdog (see the class comment)
	const bool active = context.status().nav_state == vehicle_status_s::NAVIGATION_STATE_FORMIC_TRACKER;

	if (active && !_was_active) {
		_last_output = now;
	}

	_was_active = active;

	if (active && !_output_stalled) {
		vehicle_attitude_setpoint_s attitude_setpoint{};
		_vehicle_attitude_setpoint_sub.copy(&attitude_setpoint);

		// Newest setpoint seen since entering the mode (the clock starts at
		// entry, so the previous mode's setpoints don't count). A copy can
		// also come back zeroed once the publisher is gone - that, like any
		// older timestamp, just doesn't move it.
		_last_output = math::max(attitude_setpoint.timestamp, _last_output);

		if (now > _last_output + loss_timeout) {
			_output_stalled = true;
			PX4_ERR("formic_tracker published no attitude setpoint for %.2f s, falling back to FormicAltHold",
				(double)((now - _last_output) * 1e-6f));
		}
	}

	if (_output_stalled && (!context.isArmed()
				|| context.status().nav_state_user_intention != vehicle_status_s::NAVIGATION_STATE_FORMIC_TRACKER)) {
		_output_stalled = false;
	}

	// This is a mode requirement, no need to report
	reporter.failsafeFlags().formic_tracker_signal_lost = !data_is_recent || _output_stalled;
}
