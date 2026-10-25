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

#pragma once

#include "../Common.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/topics/formic_tracker_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

// Entry/runtime gate for NAVIGATION_STATE_FORMIC_TRACKER: mirrors OffboardChecks,
// but keyed off the formic_tracker_setpoint stream itself instead of a separate
// offboard_control_mode heartbeat - the u-vector message doubles as its own
// liveness signal.
//
// It also watches the mode's output: while nav_state is FORMIC_TRACKER, the
// formic_tracker module must keep publishing vehicle_attitude_setpoint. If it
// stops for FMTRK_LOSS_T, the vehicle would otherwise keep flying the last
// attitude setpoint open-loop (seen once in SITL, when an HRT race silently
// dropped the module's interval timer), so the signal-lost flag is latched and
// commander falls back to FormicAltHold. The latch holds until the user
// selects another mode or disarms - re-entering a mode whose generator is dead
// would just repeat the open-loop window every FMTRK_LOSS_T.
class FormicTrackerChecks : public HealthAndArmingCheckBase
{
public:
	FormicTrackerChecks() = default;
	~FormicTrackerChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	uORB::Subscription _formic_tracker_setpoint_sub{ORB_ID(formic_tracker_setpoint)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};

	bool _was_active{false};          ///< nav_state == FORMIC_TRACKER on the previous check
	hrt_abstime _last_output{0};      ///< newest vehicle_attitude_setpoint seen while active (starts at mode entry)
	bool _output_stalled{false};      ///< latched: no vehicle_attitude_setpoint while active

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamFloat<px4::params::FMTRK_LOSS_T>) _param_formic_trk_loss_t
				       );
};
