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
#include <uORB/topics/formic_velo_vis_setpoint.h>
#include <uORB/topics/trajectory_setpoint.h>

// Entry/runtime gate for NAVIGATION_STATE_FORMIC_VELO_VIS: same shape as
// FormicVisServoChecks, but keyed off the formic_velo_vis_setpoint stream
// and additionally requiring a valid local velocity estimate (mc_pos_control's
// velocity controller needs it to close the loop - same requirement
// OffboardChecks applies to offboard_control_mode.velocity).
//
// It also watches the mode's output: while nav_state is FORMIC_VELO_VIS,
// this module must keep publishing trajectory_setpoint. If it stops for
// FVV_LOSS_T, mc_pos_control would otherwise keep tracking the last velocity
// setpoint open-loop, so the signal-lost flag is latched and commander falls
// back to ALTCTL. The latch holds until the user selects another mode
// or disarms - re-entering a mode whose generator is dead would just repeat
// the open-loop window every FVV_LOSS_T.
class FormicVeloVisChecks : public HealthAndArmingCheckBase
{
public:
	FormicVeloVisChecks() = default;
	~FormicVeloVisChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	uORB::Subscription _formic_velo_vis_setpoint_sub{ORB_ID(formic_velo_vis_setpoint)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};

	bool _was_active{false};          ///< nav_state == FORMIC_VELO_VIS on the previous check
	hrt_abstime _last_output{0};      ///< newest trajectory_setpoint seen while active (starts at mode entry)
	bool _output_stalled{false};      ///< latched: no trajectory_setpoint while active

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamFloat<px4::params::FVV_LOSS_T>) _param_formic_velo_vis_loss_t
				       );
};
