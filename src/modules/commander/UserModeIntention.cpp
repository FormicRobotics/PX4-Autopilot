/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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


#include "UserModeIntention.hpp"
#include <parameters/param.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

UserModeIntention::UserModeIntention(const vehicle_status_s &vehicle_status,
				     const HealthAndArmingChecks &health_and_arming_checks, ModeChangeHandler *handler)
	: _vehicle_status(vehicle_status), _health_and_arming_checks(health_and_arming_checks),
	  _handler(handler)
{
	int32_t formic_est_mode = 0;
	param_get(param_find("FORMIC_EST_MODE"), &formic_est_mode);
	_formic_vio_mode = (formic_est_mode == 1);
	_param_formic_ret_delay = param_find("FORMIC_RET_DELAY");
}

bool UserModeIntention::change(uint8_t user_intended_nav_state, ModeChangeSource source, bool allow_fallback,
			       bool force)
{
	_ever_had_mode_change = true;

	if (_handler) {
		// If a replacement mode is selected, select the internal one instead. The replacement will be selected after.
		user_intended_nav_state = _handler->getReplacedModeIfAny(user_intended_nav_state);
	}

	// Always allow mode change while disarmed
	bool always_allow = force || !isArmed();
	bool allow_change = true;

	if (!always_allow) {
		allow_change = _health_and_arming_checks.canRun(user_intended_nav_state);

		// Check fallback
		if (!allow_change && allow_fallback) {
			if (user_intended_nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL) {
				allow_change = _health_and_arming_checks.canRun(vehicle_status_s::NAVIGATION_STATE_ALTCTL);
				// We still use the original user intended mode. The failsafe state machine will then set the
				// fallback and once can_run becomes true, the actual user intended mode will be selected.
			}
		}

		///////add by naor ////////////////
		// canRun() alone is not enough for a position mode: after the EV feed is cut, EKF2 keeps
		// reporting a valid position until EKF2_NOAID_TOUT expires. Require the same EV convergence
		// gate that tick() uses, so a direct switch can't enter position mode on a stale estimate.
		if (allow_change && modeRequiresPosition(user_intended_nav_state) && !evReadyForPosition()) {
			allow_change = false;
		}

		// If change failed because position is not yet valid, park the request.
		// tick() will retry, with no timeout, until position becomes valid and EKF converges.
		if (!allow_change && modeRequiresPosition(user_intended_nav_state)) {
			// Only (re)start the wait clock when this is a *fresh* pending request, so that
			// re-issuing the same request while already pending (failsafe re-forcing a mode,
			// a GCS resending the command, ...) doesn't reset the elapsed-time log below.
			if (_pending_nav_state != user_intended_nav_state) {
				_pos_wait_start_us = hrt_absolute_time();
				PX4_INFO("Mode %d requires position - waiting for solution", user_intended_nav_state);
			}

			_pending_nav_state = user_intended_nav_state;
			// Signal immediately that we are trying to enter a position mode.
			publish_formic_pos_req(true);
		}
		///////add by naor ////////////////
	}

	// never allow to change out of termination state
	allow_change &= _vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_TERMINATION;

	if (allow_change) {
		_had_mode_change = true;
		_user_intented_nav_state = user_intended_nav_state;

		///////add by naor ////////////////
		// A successful explicit change cancels any pending position-wait.
		_pending_nav_state = UINT8_MAX;
		_pos_wait_start_us = 0;

		// Publish let_update_ev: true when entering a position mode, false otherwise.
		publish_formic_pos_req(modeRequiresPosition(user_intended_nav_state));
		///////add by naor ////////////////

		// Special case termination state: even though this mode prevents arming,
		// still don't switch out of it after disarm and thus store it in _nav_state_after_disarming.
		if (!_health_and_arming_checks.modePreventsArming(user_intended_nav_state)
		    || user_intended_nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION) {
			_nav_state_after_disarming = user_intended_nav_state;
		}

		if (_handler) {
			_handler->onUserIntendedNavStateChange(source, user_intended_nav_state);
		}
	}

	return allow_change;
}

///////add by naor ////////////////
bool UserModeIntention::modeRequiresPosition(uint8_t nav_state) const
{
	const failsafe_flags_s &flags = _health_and_arming_checks.failsafeFlags();
	const uint32_t bit = 1u << nav_state;
	return (flags.mode_req_local_position & bit) || (flags.mode_req_global_position & bit)
	       || (flags.mode_req_local_position_relaxed & bit) || (flags.mode_req_global_position_relaxed & bit);
}

void UserModeIntention::tick()
{
	if (_formic_vio_mode) {
		updateVisionRole();
	}

	///////add by naor ////////////////

	if (_pending_nav_state == UINT8_MAX) {
		return;
	}

	const bool pos_ok = _health_and_arming_checks.canRun(_pending_nav_state);

	if (pos_ok && evReadyForPosition()) {
		const float elapsed_s = hrt_elapsed_time(&_pos_wait_start_us) * 1e-6f;
		PX4_INFO("Position available - switching to pending mode %d after %.1f s", _pending_nav_state, (double)elapsed_s);
		const uint8_t mode = _pending_nav_state;
		_pending_nav_state = UINT8_MAX;

		_pos_wait_start_us = 0;
		change(mode, ModeChangeSource::User, false, true);
		publish_formic_pos_req(true);

	} else {
		// Keep retrying with no timeout: stay parked in the pending mode and keep
		// requesting position until pos_ok && ekfs_converged becomes true.
		publish_formic_pos_req(true);
	}
}


void UserModeIntention::onFailsafeNavState()
{
	// Keep requesting position as long as the *user's intention* (or a still-pending
	// position request) needs it, even if failsafe has temporarily forced the actual
	// nav_state into a non-position mode (e.g. POSCTL -> ALTCTL after a transient position
	// loss). Publishing pos_req = false here would cut the EV/vision feed to the estimator
	// (see formic_watchdog_ev), which is exactly what's needed to regain position - doing so
	// would permanently lock us out of ever recovering, since the failsafe can only clear
	// once position becomes valid again. Only stop requesting position once the user's
	// intended mode itself no longer requires it (e.g. they explicitly switched to ALTCTL/STAB).
	if (modeRequiresPosition(_user_intented_nav_state)
	    || (_pending_nav_state != UINT8_MAX && modeRequiresPosition(_pending_nav_state))) {
		publish_formic_pos_req(true);
		return;
	}

	_pending_nav_state = UINT8_MAX;
	_pos_wait_start_us = 0;
	publish_formic_pos_req(false);
}

void UserModeIntention::updateVisionRole()
{
	const hrt_abstime now = hrt_absolute_time();

	formic_vio_est_status_s vio{};
	const bool vio_fresh = _formic_vio_est_status_sub.copy(&vio) && (now < vio.timestamp + 200_ms);
	const bool offerable = vio_fresh && vio.pos_offerable;

	if (!offerable) {
		_vio_offerable_since_us = 0;

	} else if (_vio_offerable_since_us == 0) {
		_vio_offerable_since_us = now;
	}

	const bool nav_pos = modeRequiresPosition(_vehicle_status.nav_state);

	if (nav_pos) {
		_last_nav_pos_us = now;
	}

	// leaving Position: the estimator stays until the new mode has been out for a moment
	const bool mode_change_settling = (_last_nav_pos_us != 0) && (now < _last_nav_pos_us + 50_ms);

	const bool intention_pos = modeRequiresPosition(_user_intented_nav_state);
	const bool wanted = intention_pos || (_pending_nav_state != UINT8_MAX && modeRequiresPosition(_pending_nav_state));

	// after a failsafe dropped Position mode, return only once the VIO has been offerable for FORMIC_RET_DELAY
	const bool returning = isArmed() && intention_pos && !nav_pos;
	float ret_delay_s = 3.f;

	if (_param_formic_ret_delay != PARAM_INVALID) {
		param_get(_param_formic_ret_delay, &ret_delay_s);
	}

	const bool ret_ok = !returning
			    || (offerable && (now >= _vio_offerable_since_us + static_cast<hrt_abstime>(ret_delay_s * 1e6f)));

	const bool use_vision_now = nav_pos || mode_change_settling || (wanted && offerable && ret_ok);

	if (use_vision_now != _use_vision_now) {
		_use_vision_now = use_vision_now;
		publish_formic_pos_req(_pos_req_last);
	}
}

bool UserModeIntention::evReadyForPosition()
{
	if (_formic_vio_mode) {
		// Position mode flies on the VIO instance: it must be the primary already
		estimator_selector_status_s selector{};
		formic_vio_est_status_s vio{};
		const hrt_abstime now = hrt_absolute_time();

		return _estimator_selector_status_sub.copy(&selector) && (now < selector.timestamp + 1_s)
		       && (selector.primary_instance == 1)
		       && _formic_vio_est_status_sub.copy(&vio) && (now < vio.timestamp + 200_ms) && vio.healthy;
	}

	// This gate is about DEAD RECKONING on vision: on the VIO vehicle, entering a position
	// mode before EKF2 has converged onto the EV feed would fly on a stale estimate, so we
	// wait for formic_watchdog_ev to say the vision stream arrived and the filters agree.
	//
	// None of that applies when the EKF has a GNSS-backed global position - there is no
	// vision estimate to converge onto and nothing to wait for. In SITL in particular the
	// vehicle flies on GPS and formic_watchdog_ev does not run at all, so formic_ev_flag is
	// never published, the copy() below fails forever, and EVERY position-requiring mode
	// (POSCTL, AUTO_LOITER, AUTO_TAKEOFF, AUTO_LAND and FORMIC_VELO_VIS) becomes permanently
	// unavailable while armed - commander just answers "Switching to <mode> is currently not
	// available" for the whole flight. Keep the gate for the no-GNSS case it was written for.
	if (!_health_and_arming_checks.failsafeFlags().global_position_invalid) {
		return true;
	}

	formic_ev_flag_s formic_ev_flag{};

	if (!_formic_ev_flag_sub.copy(&formic_ev_flag)) {
		return false;
	}

	// formic_watchdog_ev publishes every 10 ms - a stale flag means the watchdog is not running
	const bool flag_fresh = hrt_elapsed_time(&formic_ev_flag.timestamp) < 100_ms;

	return flag_fresh && formic_ev_flag.ev_data_arrived && formic_ev_flag.ekfs_converged;
}

void UserModeIntention::publish_formic_pos_req(bool pos_requested)
{
	formic_pos_req_s pos_req{};
	pos_req.pos_req = pos_requested;
	pos_req.use_vision_now = _use_vision_now;
	pos_req.timestamp = hrt_absolute_time();
	_pos_req_last = pos_requested;
	_formic_pos_req_pub.publish(pos_req);


}
	///////add by naor ////////////////

void UserModeIntention::onDisarm()
{
	if (_handler) {
		_user_intented_nav_state = _handler->onDisarm(_nav_state_after_disarming);

	} else {
		_user_intented_nav_state = _nav_state_after_disarming;
	}
}
