/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include "DShot.h"

#include <cmath>
#include <cstdio>
#include <math.h> // NAN

#include <px4_arch/io_timer.h>

#include <px4_platform_common/sem.hpp>

char DShot::_telemetry_device[] {};
bool DShot::_telemetry_swap_rxtx{false};
px4::atomic_bool DShot::_request_telemetry_init{false};

DShot::DShot() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_mixing_output.setAllDisarmedValues(DSHOT_DISARM_VALUE);
	_mixing_output.setAllMinValues(DSHOT_MIN_THROTTLE);
	_mixing_output.setAllMaxValues(DSHOT_MAX_THROTTLE);

	// Avoid using the PWM failsafe params
	_mixing_output.setAllFailsafeValues(UINT16_MAX);
}

DShot::~DShot()
{
	// make sure outputs are off
	up_dshot_arm(false);

	perf_free(_cycle_perf);
	perf_free(_bdshot_rpm_perf);
	perf_free(_dshot_telem_perf);

	delete _telemetry;
}

int DShot::init()
{
	_output_mask = (1u << _num_outputs) - 1;

	// Getting initial parameter values
	update_params();

	ScheduleNow();

	return OK;
}

int DShot::task_spawn(int argc, char *argv[])
{
	DShot *instance = new DShot();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void DShot::enable_dshot_outputs(const bool enabled)
{
	if (enabled && !_outputs_initialized) {
		unsigned int dshot_frequency = 0;
		uint32_t dshot_frequency_param = 0;

		for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
			uint32_t channels = io_timer_get_group(timer);

			if (channels == 0) {
				continue;
			}

			char param_name[17];
			snprintf(param_name, sizeof(param_name), "%s_TIM%u", _mixing_output.paramPrefix(), timer);

			int32_t tim_config = 0;
			param_t handle = param_find(param_name);
			param_get(handle, &tim_config);
			unsigned int dshot_frequency_request = 0;

			if (tim_config == -5) {
				dshot_frequency_request = DSHOT150;

			} else if (tim_config == -4) {
				dshot_frequency_request = DSHOT300;

			} else if (tim_config == -3) {
				dshot_frequency_request = DSHOT600;

			} else {
				_output_mask &= ~channels; // don't use for dshot
			}

			if (dshot_frequency_request != 0) {
				if (dshot_frequency != 0 && dshot_frequency != dshot_frequency_request) {
					PX4_WARN("Only supporting a single frequency, adjusting param %s", param_name);
					param_set_no_notification(handle, &dshot_frequency_param);

				} else {
					dshot_frequency = dshot_frequency_request;
					dshot_frequency_param = tim_config;
				}
			}
		}

		_bidirectional_dshot_enabled = _param_bidirectional_enable.get();

		int ret = up_dshot_init(_output_mask, dshot_frequency, _bidirectional_dshot_enabled);

		if (ret < 0) {
			PX4_ERR("up_dshot_init failed (%i)", ret);
			return;
		}

		_output_mask = ret;

		// disable unused functions
		for (unsigned i = 0; i < _num_outputs; ++i) {
			if (((1 << i) & _output_mask) == 0) {
				_mixing_output.disableFunction(i);

			}
		}

		if (_output_mask == 0) {
			// exit the module if no outputs used
			request_stop();
			return;
		}

		_outputs_initialized = true;
	}

	if (_outputs_initialized) {
		up_dshot_arm(enabled);
		_outputs_on = enabled;
	}
}

void DShot::update_num_motors()
{
	int motor_count = 0;

	for (unsigned i = 0; i < _num_outputs; ++i) {
		if (_mixing_output.isFunctionSet(i)) {
			_actuator_functions[motor_count] = (uint8_t)_mixing_output.outputFunction(i);
			++motor_count;
		}
	}

	_num_motors = motor_count;
}

void DShot::init_telemetry(const char *device, bool swap_rxtx)
{
	if (!_telemetry) {
		_telemetry = new DShotTelemetry{};

		if (!_telemetry) {
			PX4_ERR("alloc failed");
			return;
		}
	}

	if (device != NULL) {
		int ret = _telemetry->init(device, swap_rxtx);

		if (ret != 0) {
			PX4_ERR("telemetry init failed (%i)", ret);
		}
	}

	update_num_motors();
}

int DShot::handle_new_telemetry_data(const int telemetry_index, const DShotTelemetry::EscData &data, bool ignore_rpm)
{
	int ret = 0;
	// fill in new motor data
	esc_status_s &esc_status = esc_status_pub.get();

	if (telemetry_index < esc_status_s::CONNECTED_ESC_MAX) {
		esc_status.esc_online_flags |= 1 << telemetry_index;

		esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];

		if (!ignore_rpm) {
			// If we also have bidirectional dshot, we use rpm and timestamps from there.
			esc_status.esc[telemetry_index].timestamp       = data.time;
			esc_status.esc[telemetry_index].esc_rpm         = (static_cast<int>(data.erpm) * 100) /
					(_param_mot_pole_count.get() / 2);
		}

		esc_status.esc[telemetry_index].esc_voltage     = static_cast<float>(data.voltage) * 0.01f;
		esc_status.esc[telemetry_index].esc_current     = static_cast<float>(data.current) * 0.01f;
		esc_status.esc[telemetry_index].esc_temperature = static_cast<float>(data.temperature);
		// TODO: accumulate consumption and use for battery estimation
	}

	// publish when motor index wraps (which is robust against motor timeouts)
	if (telemetry_index <= _last_telemetry_index) {
		esc_status.timestamp = hrt_absolute_time();
		esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
		esc_status.esc_count = _num_motors;
		++esc_status.counter;

		ret = 1; // Indicate we wrapped, so we publish data
	}

	_last_telemetry_index = telemetry_index;

	perf_count(_dshot_telem_perf);

	return ret;
}

void DShot::publish_esc_status(void)
{
	esc_status_s &esc_status = esc_status_pub.get();
	int telemetry_index = 0;

	// clear data of the esc that are offline
	for (int index = 0; (index < _last_telemetry_index); index++) {
		if ((esc_status.esc_online_flags & (1 << index)) == 0) {
			memset(&esc_status.esc[index], 0, sizeof(struct esc_report_s));
		}
	}

	// FIXME: mark all UART Telemetry ESC's as online, otherwise commander complains even for a single dropout
	esc_status.esc_count = _num_motors;
	esc_status.esc_online_flags = (1 << esc_status.esc_count) - 1;
	esc_status.esc_armed_flags = (1 << esc_status.esc_count) - 1;

	if (_bidirectional_dshot_enabled) {
		for (unsigned i = 0; i < _num_outputs; i++) {
			if (_mixing_output.isFunctionSet(i)) {
				if (up_bdshot_channel_status(i)) {
					esc_status.esc_online_flags |= 1 << i;

				} else {
					esc_status.esc_online_flags &= ~(1 << i);
				}

				++telemetry_index;
			}
		}
	}

	if (!esc_status_pub.advertised()) {
		esc_status_pub.advertise();

	} else {
		esc_status_pub.update();
	}

	// reset esc online flags
	esc_status.esc_online_flags = 0;
}

int DShot::handle_new_bdshot_erpm(void)
{
	int num_erpms = 0;
	int telemetry_index = 0;
	int erpm;
	esc_status_s &esc_status = esc_status_pub.get();

	esc_status.timestamp = hrt_absolute_time();
	esc_status.counter = _esc_status_counter++;
	esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_DSHOT;
	esc_status.esc_armed_flags = _outputs_on;

	// We wait until all are ready.
	if (up_bdshot_num_erpm_ready() < _num_motors) {
		return 0;
	}

	for (unsigned i = 0; i < _num_outputs; i++) {
		if (_mixing_output.isFunctionSet(i)) {
			if (up_bdshot_get_erpm(i, &erpm) == 0) {
				num_erpms++;
				esc_status.esc_online_flags |= 1 << telemetry_index;
				esc_status.esc[telemetry_index].timestamp = hrt_absolute_time();
				esc_status.esc[telemetry_index].esc_rpm = (erpm * 100) / (_param_mot_pole_count.get() / 2);
				esc_status.esc[telemetry_index].actuator_function = _actuator_functions[telemetry_index];
			}

			++telemetry_index;
		}
	}

	perf_count(_bdshot_rpm_perf);

	return num_erpms;
}

int DShot::send_command_thread_safe(const dshot_command_t command, const int num_repetitions, const int motor_index)
{
	Command cmd{};
	cmd.command = command;

	if (motor_index == -1) {
		cmd.motor_mask = 0xff;

	} else {
		cmd.motor_mask = 1 << motor_index;
	}

	cmd.num_repetitions = num_repetitions;
	_new_command.store(&cmd);

	hrt_abstime timestamp_for_timeout = hrt_absolute_time();

	// wait until main thread processed it
	while (_new_command.load()) {

		if (hrt_elapsed_time(&timestamp_for_timeout) < 2_s) {
			px4_usleep(1000);

		} else {
			_new_command.store(nullptr);
			PX4_WARN("DShot command timeout!");
		}
	}

	return 0;
}

void DShot::mixerChanged()
{
	update_num_motors();
}

bool DShot::updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			  unsigned num_outputs, unsigned num_control_groups_updated)
{
	if (!_outputs_on) {
		return false;
	}

	// ---- Turtle recovery: may take over the outputs while disarmed ----------
	_trtl_output_seen = true;
	update_turtle_inputs(); // the mixer has consumed the latest arming/test state by this callback

	if (_trtl_core.owns_outputs() && _current_command.valid()) {
		// A queued DShot command must not run while direction is temporarily reversed.
		_current_command.num_repetitions = 0;
		_trtl_input.hardware_ok = false;
		PX4_WARN("turtle: cancelled concurrent DShot command");
	}

	_trtl_frame = _trtl_core.step(hrt_absolute_time(), _trtl_input);

	// Do not let a motor-test path bypass startup direction normalisation.
	if (_trtl_frame.action == turtle::Action::NORMAL && _trtl_requested
	    && (!_trtl_core.configured() || !_trtl_core.synced())
	    && !(_mixing_output.armed().armed || _trtl_armed.armed)
	    && turtle::interlock().owner() != turtle::Owner::FLIGHT) {
		_trtl_frame = {turtle::Action::STOP, 0, 0};
	}

	if (_trtl_frame.action != turtle::Action::NORMAL) {
		for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
			const uint32_t bit = uint32_t{1} << i;

			if (!(_output_mask & bit)) { continue; }

			if (!(_trtl_frame.outputs & bit)) {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, false);

			} else if (_trtl_frame.action == turtle::Action::REVERSE) {
				up_dshot_motor_command(i, DShot_cmd_spin_direction_reversed, true);

			} else if (_trtl_frame.action == turtle::Action::RESTORE) {
				up_dshot_motor_command(i, DShot_cmd_spin_direction_normal, true);

			} else if (_trtl_frame.action == turtle::Action::THROTTLE) {
				up_dshot_motor_data_set(i, _trtl_frame.level, false);

			} else {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, false);
			}
		}

		up_dshot_trigger();
		return true;
	}

	if (_trtl_fallback) {
		// Fallback tick from Run(): never manufacture normal/test/flight values.
		return false;
	}

	// ---- end turtle recovery -----------------------------------------------

	int requested_telemetry_index = -1;

	if (_telemetry) {
		requested_telemetry_index = _telemetry->getRequestMotorIndex();
	}

	int telemetry_index = 0;

	for (int i = 0; i < (int)num_outputs; i++) {

		uint16_t output = outputs[i];

		if (output == DSHOT_DISARM_VALUE) {

			if (_current_command.valid() && (_current_command.motor_mask & (1 << i))) {
				up_dshot_motor_command(i, _current_command.command, true);

			} else {
				up_dshot_motor_command(i, DShot_cmd_motor_stop, telemetry_index == requested_telemetry_index);
			}

		} else {

			if (_param_dshot_3d_enable.get() || (_reversible_outputs & (1u << i))) {
				output = convert_output_to_3d_scaling(output);
			}

			up_dshot_motor_data_set(i, math::min(output, static_cast<uint16_t>(DSHOT_MAX_THROTTLE)),
						telemetry_index == requested_telemetry_index);
		}

		telemetry_index += _mixing_output.isFunctionSet(i);
	}

	// Decrement the command counter
	if (_current_command.valid()) {
		--_current_command.num_repetitions;

		// Queue a save command after the burst if save has been requested
		if (_current_command.num_repetitions == 0 && _current_command.save) {
			_current_command.save = false;
			_current_command.num_repetitions = 10;
			_current_command.command = dshot_command_t::DShot_cmd_save_settings;
		}
	}

	up_dshot_trigger();

	return true;
}

uint16_t DShot::convert_output_to_3d_scaling(uint16_t output)
{
	// DShot 3D splits the throttle ranges in two.
	// This is in terms of DShot values, code below is in terms of actuator_output
	// Direction 1) 48 is the slowest, 1047 is the fastest.
	// Direction 2) 1049 is the slowest, 2047 is the fastest.
	if (output >= _param_dshot_3d_dead_l.get() && output < _param_dshot_3d_dead_h.get()) {
		return DSHOT_DISARM_VALUE;
	}

	bool upper_range = output >= 1000;

	if (upper_range) {
		output -= 1000;

	} else {
		output = 999 - output; // lower range is inverted
	}

	float max_output = 999.f;
	float min_output = max_output * _param_dshot_min.get();
	output = math::min(max_output, (min_output + output * (max_output - min_output) / max_output));

	if (upper_range) {
		output += 1000;
	}

	return output;
}

void DShot::update_turtle_parameters()
{
	using namespace turtle;
	const uint64_t now = hrt_absolute_time();

	// Parameters are read at 4 Hz at most; never during a motor burst.
	if (_trtl_last_params && now >= _trtl_last_params && now - _trtl_last_params < 250_ms) { return; }

	_trtl_last_params = now;

	if (_trtl_enable_handle == PARAM_INVALID) { _trtl_enable_handle = param_find("TRTL_EN"); }

	int32_t requested = 0;

	if (_trtl_enable_handle != PARAM_INVALID && param_get(_trtl_enable_handle, &requested) == PX4_OK) {
		_trtl_requested = requested != 0;
	}

	// Accept changes only while idle, disarmed, and (once loaded) with the switch OFF.
	// The snapshot is frozen for the whole recovery including direction restoration.
	if (_trtl_core.busy() || interlock().owner() == Owner::FLIGHT) { return; }

	if (!fresh(now, _trtl_status.timestamp, 750_ms) || !fresh(now, _trtl_armed.timestamp, 750_ms)
	    || _trtl_status.arming_state != vehicle_status_s::ARMING_STATE_DISARMED
	    || _trtl_armed.armed || _mixing_output.armed().armed) {
		return;
	}

	if (_trtl_loaded && !_trtl_input.switch_off) { return; }

	static const char *const names[TRTL_PARAM_COUNT] = {
		"TRTL_EN", "TRTL_ESC", "TRTL_CH", "TRTL_POL",
		"TRTL_ANGLE", "TRTL_EXIT", "TRTL_MIN", "TRTL_MAX", "TRTL_RAMP", "TRTL_TIME",
		"TRTL_PRE", "TRTL_POST", "TRTL_SETTLE", "TRTL_COOL", "TRTL_DB", "TRTL_PWR", "TRTL_HOLD",
		"TRTL_FL", "TRTL_FR", "TRTL_RL", "TRTL_RR", "CA_ROTOR_COUNT"
	};

	if (!_trtl_handles_initialized) {
		for (auto &h : _trtl_handles) { h = PARAM_INVALID; }

		_trtl_handles_initialized = true;
	}

	int32_t v[TRTL_PARAM_COUNT] {};

	for (unsigned i = 0; i < TRTL_PARAM_COUNT; ++i) {
		if (_trtl_handles[i] == PARAM_INVALID) { _trtl_handles[i] = param_find(names[i]); }

		if (_trtl_handles[i] == PARAM_INVALID || param_get(_trtl_handles[i], &v[i]) != PX4_OK) {
			_trtl_loaded = false;
			_trtl_core.invalidate();
			return;
		}
	}

	Config c;
	c.enabled = v[0];
	c.esc = v[1];
	c.channel = v[2];
	c.polarity = v[3];
	c.entry_deg = v[4];
	c.exit_deg = v[5];
	c.start = v[6];
	c.peak = v[7];
	c.ramp_ms = v[8];
	c.drive_ms = v[9];
	c.pre_ms = v[10];
	c.post_ms = v[11];
	c.settle_ms = v[12];
	c.cooldown_ms = v[13];
	c.deadband_pct = v[14];
	c.proportional = v[15];
	c.hold_ms = v[16];

	turtle_mapping::Config corners;

	for (unsigned i = 0; i < 4; ++i) { corners.motor[i] = v[17 + i]; }

	int32_t functions[MAX_ACTUATORS] {};
	static_assert(MAX_ACTUATORS <= 32, "turtle output mask too narrow");

	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		functions[i] = static_cast<int32_t>(_mixing_output.outputFunction(i));
	}

	const auto resolved = turtle_mapping::resolve(corners, functions, MAX_ACTUATORS, _output_mask);
	c.outputs = resolved.all;
	_trtl_geometry_ok = v[21] == 4 && resolved.valid();

	// Validate the allocator geometry of each corner, not only the rotor count.
	static const char *const suffix[] = {"PX", "PY", "AX", "AY", "AZ"};

	for (unsigned i = 0; i < 4 && _trtl_geometry_ok; ++i) {
		float geom[5] {};

		for (unsigned j = 0; j < 5; ++j) {
			char name[24];
			snprintf(name, sizeof(name), "CA_ROTOR%ld_%s", long(corners.motor[i] - 1), suffix[j]);
			const param_t p = param_find(name);

			if (p == PARAM_INVALID || param_get(p, &geom[j]) != PX4_OK) { _trtl_geometry_ok = false; break; }
		}

		_trtl_geometry_ok = _trtl_geometry_ok && quad_corner(i, geom[0], geom[1], geom[2], geom[3], geom[4]);

		char tilt_name[32];
		snprintf(tilt_name, sizeof(tilt_name), "CA_ROTOR%ld_TILT", long(corners.motor[i] - 1));
		const param_t tilt_handle = param_find(tilt_name);
		int32_t tilt = 0;

		if (tilt_handle == PARAM_INVALID || param_get(tilt_handle, &tilt) != PX4_OK || tilt != 0) {
			_trtl_geometry_ok = false;
		}
	}

	_trtl_rc_conflict = rc_conflict(_trtl_rc_channels.function,
					sizeof(_trtl_rc_channels.function) / sizeof(_trtl_rc_channels.function[0]), c.channel);

	// Mode selection may be configured outside rc_channels.function.
	static const char *const mode_names[] = {"RC_MAP_FLTMODE", "RC_MAP_MODE_SW", "RC_MAP_FLTBTN"};

	for (const auto name : mode_names) {
		const param_t p = param_find(name);
		int32_t ch = 0;

		if (p != PARAM_INVALID && param_get(p, &ch) == PX4_OK && ch == c.channel) { _trtl_rc_conflict = true; }
	}

	const bool raw_ok = rc::healthy(now, _trtl_rc_channels.timestamp, _trtl_rc_channels.timestamp_last_valid,
					_trtl_rc_channels.signal_lost);
	const float raw = raw_ok ? rc::select(_trtl_rc_channels.channels, _trtl_rc_channels.channel_count, c.channel) : NAN;

	// Only latch a new snapshot with the switch clearly OFF.
	if (!std::isfinite(raw) || raw * float(c.polarity) >= -0.3f) { return; }

	_trtl_loaded = false;

	if (!resolved.valid() || !_trtl_geometry_ok || _trtl_rc_conflict || !c.valid()) {
		_trtl_core.invalidate();
		return;
	}

	// A changed mapping invalidates synchronisation even if the same output bits are reused.
	if (_trtl_core.configured() && !turtle_mapping::same(_trtl_map, resolved)) { _trtl_core.invalidate(); }

	_trtl_config = c;
	_trtl_map_config = corners;
	_trtl_map = resolved;
	_trtl_loaded = _trtl_core.configure(c);
}

void DShot::update_turtle_inputs()
{
	using namespace turtle;

	if (_trtl_manual_sub.updated()) { _trtl_manual_sub.copy(&_trtl_manual); }

	if (_trtl_status_sub.updated()) { _trtl_status_sub.copy(&_trtl_status); }

	if (_trtl_armed_sub.updated()) { _trtl_armed_sub.copy(&_trtl_armed); }

	if (_trtl_att_sub.updated()) { _trtl_att_sub.copy(&_trtl_att); }

	if (_trtl_rc_sub.updated()) { _trtl_rc_sub.copy(&_trtl_rc_channels); }

	const uint64_t now = hrt_absolute_time();
	Input in;

	const bool state_fresh = fresh(now, _trtl_status.timestamp, 750_ms) && fresh(now, _trtl_armed.timestamp, 750_ms);
	in.disarmed = state_fresh && _trtl_status.arming_state == vehicle_status_s::ARMING_STATE_DISARMED
		      && !_trtl_armed.armed && !_mixing_output.armed().armed;

	const auto &a = _mixing_output.armed();
	in.hardware_ok = state_fresh && !_trtl_armed.lockdown && !_trtl_armed.kill
			 && !_trtl_armed.termination && !_trtl_armed.in_esc_calibration_mode
			 && !a.lockdown && !a.kill && !a.termination && !a.in_esc_calibration_mode
			 && (!_trtl_status.safety_button_available || _trtl_status.safety_off)
			 && !_trtl_status.calibration_enabled && !_trtl_status.rc_calibration_in_progress
			 && !_trtl_status.failsafe && !_mixing_output.isActuatorTestRunning() && !_current_command.valid();

	int32_t functions[MAX_ACTUATORS] {};

	for (unsigned i = 0; i < MAX_ACTUATORS; ++i) {
		functions[i] = static_cast<int32_t>(_mixing_output.outputFunction(i));
	}

	const auto current = turtle_mapping::resolve(_trtl_map_config, functions, MAX_ACTUATORS, _output_mask);
	_trtl_map_ok = _trtl_loaded && _trtl_geometry_ok && turtle_mapping::same(current, _trtl_map)
		       && !_trtl_status.is_vtol && _trtl_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		       && !_param_dshot_3d_enable.get() && !_reversible_outputs;
	in.mapping_ok = _trtl_map_ok;

	const bool raw_ok = rc::healthy(now, _trtl_rc_channels.timestamp, _trtl_rc_channels.timestamp_last_valid,
					_trtl_rc_channels.signal_lost);
	const bool conflict = rc_conflict(_trtl_rc_channels.function,
					  sizeof(_trtl_rc_channels.function) / sizeof(_trtl_rc_channels.function[0]),
					  _trtl_config.channel);
	in.rc_ok = raw_ok && !conflict && !_trtl_rc_conflict && fresh(now, _trtl_manual.timestamp, 300_ms)
		   && fresh(now, _trtl_manual.timestamp_sample, 300_ms) && _trtl_manual.valid
		   && _trtl_manual.data_source == manual_control_setpoint_s::SOURCE_RC;

	const float raw = raw_ok ? rc::select(_trtl_rc_channels.channels, _trtl_rc_channels.channel_count,
					      _trtl_config.channel) : NAN;
	const float sw = raw * float(_trtl_config.polarity);
	in.switch_on = in.rc_ok && std::isfinite(sw) && sw > 0.7f;
	// Centre is neither ON nor a valid OFF reset; an ambiguous switch stops a burst.
	in.switch_off = in.rc_ok && std::isfinite(sw) && sw < -0.3f;
	in.throttle_low = in.rc_ok && std::isfinite(_trtl_manual.throttle) && _trtl_manual.throttle < -0.85f;
	in.vertical = vertical(_trtl_att.q);
	in.attitude_ok = fresh(now, _trtl_att.timestamp, 300_ms) && std::isfinite(in.vertical);

	const float roll = _trtl_manual.roll, pitch = _trtl_manual.pitch;
	const bool sticks = std::isfinite(roll) && std::isfinite(pitch) && std::fabs(roll) <= 1.05f && std::fabs(pitch) <= 1.05f;

	if (sticks) {
		in.selected = turtle_mapping::select(_trtl_map, roll, pitch, float(_trtl_config.deadband_pct) * 0.01f);
		const float roll_magnitude = std::fabs(roll);
		const float pitch_magnitude = std::fabs(pitch);
		in.strength = (roll_magnitude > pitch_magnitude) ? roll_magnitude : pitch_magnitude;

		if (in.strength > 1.f) { in.strength = 1.f; }

	} else {
		in.rc_ok = false;
	}

	_trtl_input = in;
}

void DShot::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	update_turtle_parameters();
	update_turtle_inputs();
	_trtl_output_seen = false;

	_mixing_output.update();

	// Turtle recovery needs a steady output cadence even if the mixer did not
	// call updateOutputs() this cycle (e.g. no fresh actuator topic while disarmed).
	if (!_trtl_output_seen && (_trtl_requested || _trtl_core.owns_outputs()) && _outputs_on) {
		uint16_t turtle_zero[MAX_ACTUATORS] {};
		_trtl_fallback = true;
		updateOutputs(turtle_zero, MAX_ACTUATORS, 0);
		_trtl_fallback = false;
	}

	if (_trtl_requested || _trtl_core.owns_outputs()) {
		ScheduleDelayed(10_ms);
	}

	// update output status if armed or if mixer is loaded
	bool outputs_on = true;

	if (_outputs_on != outputs_on) {
		enable_dshot_outputs(outputs_on);
	}

	if (_telemetry) {
		const int telem_update = _telemetry->update(_num_motors);

		if (telem_update >= 0) {
			const int need_to_publish = handle_new_telemetry_data(telem_update, _telemetry->latestESCData(),
						    _bidirectional_dshot_enabled);

			// We don't want to publish twice, once by telemetry and once by bidirectional dishot.
			if (!_bidirectional_dshot_enabled && need_to_publish) {
				publish_esc_status();
			}
		}
	}

	if (_bidirectional_dshot_enabled) {
		// Add bdshot data to esc status
		const int need_to_publish = handle_new_bdshot_erpm();

		if (need_to_publish) {
			publish_esc_status();
		}
	}

	if (_parameter_update_sub.updated()) {
		update_params();
	}

	// telemetry device update request?
	if (_request_telemetry_init.load()) {
		init_telemetry(_telemetry_device, _telemetry_swap_rxtx);
		_request_telemetry_init.store(false);
	}

	// new command?
	if (!_current_command.valid()) {
		Command *new_command = _new_command.load();

		if (new_command) {
			_current_command = *new_command;
			_new_command.store(nullptr);
		}
	}

	handle_vehicle_commands();

	if (!_mixing_output.armed().armed) {
		if (_reversible_outputs != _mixing_output.reversibleOutputs()) {
			_reversible_outputs = _mixing_output.reversibleOutputs();
			update_params();
		}
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true);

	perf_end(_cycle_perf);
}

void DShot::handle_vehicle_commands()
{
	vehicle_command_s vehicle_command;

	while (!_current_command.valid() && _vehicle_command_sub.update(&vehicle_command)) {

		if (vehicle_command.command == vehicle_command_s::VEHICLE_CMD_CONFIGURE_ACTUATOR) {
			int function = (int)(vehicle_command.param5 + 0.5);

			if (function < 1000) {
				const int first_motor_function = 1; // from MAVLink ACTUATOR_OUTPUT_FUNCTION
				const int first_servo_function = 33;

				if (function >= first_motor_function && function < first_motor_function + actuator_test_s::MAX_NUM_MOTORS) {
					function = function - first_motor_function + actuator_test_s::FUNCTION_MOTOR1;

				} else if (function >= first_servo_function && function < first_servo_function + actuator_test_s::MAX_NUM_SERVOS) {
					function = function - first_servo_function + actuator_test_s::FUNCTION_SERVO1;

				} else {
					function = INT32_MAX;
				}

			} else {
				function -= 1000;
			}

			int type = (int)(vehicle_command.param1 + 0.5f);
			int index = -1;

			for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
				if ((int)_mixing_output.outputFunction(i) == function) {
					index = i;
				}
			}

			vehicle_command_ack_s command_ack{};
			command_ack.command = vehicle_command.command;
			command_ack.target_system = vehicle_command.source_system;
			command_ack.target_component = vehicle_command.source_component;
			command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

			if (index != -1) {
				PX4_DEBUG("setting command: index: %i type: %i", index, type);
				_current_command.command = dshot_command_t::DShot_cmd_motor_stop;

				switch (type) {
				case 1: _current_command.command = dshot_command_t::DShot_cmd_beacon1; break;

				case 2: _current_command.command = dshot_command_t::DShot_cmd_3d_mode_on; break;

				case 3: _current_command.command = dshot_command_t::DShot_cmd_3d_mode_off; break;

				case 4: _current_command.command = dshot_command_t::DShot_cmd_spin_direction_1; break;

				case 5: _current_command.command = dshot_command_t::DShot_cmd_spin_direction_2; break;
				}

				if (_current_command.command == dshot_command_t::DShot_cmd_motor_stop) {
					PX4_WARN("unknown command: %i", type);

				} else {
					command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					_current_command.motor_mask = 1 << index;
					_current_command.num_repetitions = 10;
					_current_command.save = true;
				}

			}

			command_ack.timestamp = hrt_absolute_time();
			_command_ack_pub.publish(command_ack);
		}
	}
}

void DShot::update_params()
{
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();

	// we use a minimum value of 1, since 0 is for disarmed
	_mixing_output.setAllMinValues(math::constrain(static_cast<int>((_param_dshot_min.get() *
				       static_cast<float>(DSHOT_MAX_THROTTLE))),
				       DSHOT_MIN_THROTTLE, DSHOT_MAX_THROTTLE));

	// Do not use the minimum parameter for reversible outputs
	for (unsigned i = 0; i < _num_outputs; ++i) {
		if ((1 << i) & _reversible_outputs) {
			_mixing_output.minValue(i) = DSHOT_MIN_THROTTLE;
		}
	}
}

int DShot::custom_command(int argc, char *argv[])
{
	if (turtle::interlock().recovery_owned()) {
		PX4_WARN("DShot command denied during turtle recovery/restoration");
		return PX4_ERROR;
	}

	const char *verb = argv[0];

	int motor_index = -1; // select motor index, default: -1=all
	int myoptind = 1;
	bool swap_rxtx = false;
	const char *device_name = nullptr;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "m:xd:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'm':
			motor_index = strtol(myoptarg, nullptr, 10) - 1;
			break;

		case 'x':
			swap_rxtx = true;
			break;

		case 'd':
			device_name = myoptarg;
			break;

		default:
			return print_usage("unrecognized flag");
		}
	}

	if (!strcmp(verb, "telemetry")) {
		if (device_name) {
			// telemetry can be requested before the module is started
			strncpy(_telemetry_device, device_name, sizeof(_telemetry_device) - 1);
			_telemetry_device[sizeof(_telemetry_device) - 1] = '\0';
			_telemetry_swap_rxtx = swap_rxtx;
			_request_telemetry_init.store(true);
		}

		return 0;
	}

	struct VerbCommand {
		const char *name;
		dshot_command_t command;
		int num_repetitions;
	};

	constexpr VerbCommand commands[] = {
		{"reverse", DShot_cmd_spin_direction_2, 10},
		{"normal", DShot_cmd_spin_direction_1, 10},
		{"save", DShot_cmd_save_settings, 10},
		{"3d_on", DShot_cmd_3d_mode_on, 10},
		{"3d_off", DShot_cmd_3d_mode_off, 10},
		{"beep1", DShot_cmd_beacon1, 1},
		{"beep2", DShot_cmd_beacon2, 1},
		{"beep3", DShot_cmd_beacon3, 1},
		{"beep4", DShot_cmd_beacon4, 1},
		{"beep5", DShot_cmd_beacon5, 1},
	};

	for (unsigned i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
		if (!strcmp(verb, commands[i].name)) {
			if (!is_running()) {
				PX4_ERR("module not running");
				return -1;
			}

			return get_instance()->send_command_thread_safe(commands[i].command, commands[i].num_repetitions, motor_index);
		}
	}

	if (!is_running()) {
		int ret = DShot::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int DShot::print_status()
{
	PX4_INFO("Outputs initialized: %s", _outputs_initialized ? "yes" : "no");
	PX4_INFO("Outputs used: 0x%" PRIx32, _output_mask);
	PX4_INFO("Outputs on: %s", _outputs_on ? "yes" : "no");
	PX4_INFO("3D mode parameter (DSHOT_3D_ENABLE): %s", _param_dshot_3d_enable.get() ? "enabled" : "disabled");

	PX4_INFO("Turtle: requested=%d configured=%d enabled=%d synced=%d phase=%s reason=%s",
		 (int)_trtl_requested, (int)_trtl_loaded, (int)_trtl_core.enabled(), (int)_trtl_core.synced(),
		 turtle::phase_name(_trtl_core.phase), turtle::reason_name(_trtl_core.reason));
	PX4_INFO("Turtle: map=%d geometry=%d rc_conflict=%d owner=%u ready=%d",
		 (int)_trtl_map_ok, (int)_trtl_geometry_ok, (int)_trtl_rc_conflict,
		 (unsigned)turtle::interlock().owner(), (int)turtle::interlock().ready());
	PX4_INFO("Turtle: outputs FL=%u FR=%u RL=%u RR=%u CH=%ld POL=%ld",
		 turtle_mapping::output_number(_trtl_map.corner[0]), turtle_mapping::output_number(_trtl_map.corner[1]),
		 turtle_mapping::output_number(_trtl_map.corner[2]), turtle_mapping::output_number(_trtl_map.corner[3]),
		 long(_trtl_config.channel), long(_trtl_config.polarity));
	PX4_INFO("Turtle: entry=%ld exit=%ld start=%ld peak=%ld ramp=%ld drive=%ld",
		 long(_trtl_config.entry_deg), long(_trtl_config.exit_deg), long(_trtl_config.start),
		 long(_trtl_config.peak), long(_trtl_config.ramp_ms), long(_trtl_config.drive_ms));
	PX4_INFO("Turtle: attempts=%u restores_sent=%u reverse_frames=%u restore_frames=%u throttle_frames=%u last_drive_ms=%u",
		 (unsigned)_trtl_core.attempts, (unsigned)_trtl_core.restore_sequences_sent, (unsigned)_trtl_core.reverse_frames,
		 (unsigned)_trtl_core.restore_frames, (unsigned)_trtl_core.throttle_frames, (unsigned)_trtl_core.last_drive_ms);
	PX4_INFO("Turtle: rc=%d on=%d off=%d thr_low=%d disarmed=%d hardware=%d vertical=%.3f selected=0x%x level=%u",
		 (int)_trtl_input.rc_ok, (int)_trtl_input.switch_on, (int)_trtl_input.switch_off, (int)_trtl_input.throttle_low,
		 (int)_trtl_input.disarmed, (int)_trtl_input.hardware_ok, (double)_trtl_input.vertical,
		 (unsigned)_trtl_frame.outputs, (unsigned)_trtl_frame.level);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_bdshot_rpm_perf);
	perf_print_counter(_dshot_telem_perf);

	_mixing_output.printStatus();

	if (_telemetry) {
		PX4_INFO("telemetry on: %s", _telemetry_device);
		_telemetry->printStatus();
	}

	/* Print dshot status */
	if (_bidirectional_dshot_enabled) {
		up_bdshot_status();
	}

	return 0;
}

int DShot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the DShot output driver. It is similar to the fmu driver, and can be used as drop-in replacement
to use DShot as ESC communication protocol instead of PWM.

On startup, the module tries to occupy all available pins for DShot output.
It skips all pins already in use (e.g. by a camera trigger module).

It supports:
- DShot150, DShot300, DShot600
- telemetry via separate UART and publishing as esc_status message
- sending DShot commands via CLI

### Examples
Permanently reverse motor 1:
$ dshot reverse -m 1
$ dshot save -m 1
After saving, the reversed direction will be regarded as the normal one. So to reverse again repeat the same commands.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_COMMAND_DESCR("telemetry", "Enable Telemetry on a UART");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "UART device", false);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Swap RX/TX pins", true);

	// DShot commands
	PRINT_MODULE_USAGE_COMMAND_DESCR("reverse", "Reverse motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("normal", "Normal motor direction");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("save", "Save current settings");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_on", "Enable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("3d_off", "Disable 3D mode");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep1", "Send Beep pattern 1");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep2", "Send Beep pattern 2");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep3", "Send Beep pattern 3");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep4", "Send Beep pattern 4");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("beep5", "Send Beep pattern 5");
	PRINT_MODULE_USAGE_PARAM_INT('m', -1, 0, 16, "Motor index (1-based, default=all)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int dshot_main(int argc, char *argv[])
{
	return DShot::main(argc, argv);
}
