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

#include "formic_strobe_capture.hpp"

#include <lib/parameters/param.h>
#include <px4_platform_common/log.h>

#include <board_config.h>

FormicStrobeCapture::FormicStrobeCapture() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_strobe_pub.advertise();
}

FormicStrobeCapture::~FormicStrobeCapture()
{
	if (_capturing) {
		up_input_capture_set(_channel, Disabled, 0, nullptr, nullptr);
	}

	ScheduleClear();
}

bool FormicStrobeCapture::init()
{
	for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS && _channel == -1; ++i) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_%s%d", PARAM_PREFIX, "FUNC", i + 1);
		const param_t function_handle = param_find(param_name);
		int32_t function = 0;

		if (function_handle != PARAM_INVALID && param_get(function_handle, &function) == 0
		    && function == OUTPUT_FUNCTION) {
			_channel = i;
		}
	}

	if (_channel == -1) {
		PX4_WARN("no channel set to Formic_Tracker_Strobe (%" PRId32 ")", OUTPUT_FUNCTION);
		return false;
	}

	int32_t rising = 1;
	param_get(param_find("FSTRB_EDGE"), &rising);

	// All channels of one timer share one mode: this fails when another channel of the
	// same timer already drives an output.
	const int ret = up_input_capture_set(_channel, rising ? Rising : Falling, 0,
					     &FormicStrobeCapture::capture_trampoline, this);

	if (ret != 0) {
		PX4_ERR("capture setup failed on channel %d (%d)", _channel + 1, ret);
		return false;
	}

	_capturing = true;

	// Edges schedule Run() themselves; the slow pass only lets a stop request through
	// when no strobe arrives.
	ScheduleOnInterval(500_ms);
	return true;
}

void FormicStrobeCapture::capture_trampoline(void *context, uint32_t chan_index, hrt_abstime edge_time,
		uint32_t edge_state, uint32_t overflow)
{
	static_cast<FormicStrobeCapture *>(context)->capture_callback(edge_time);
}

void FormicStrobeCapture::capture_callback(hrt_abstime edge_time)
{
	// Interrupt context.
	if ((edge_time - _last_edge_time) < MIN_EDGE_INTERVAL) {
		if (++_rate_fault_counter > MAX_RATE_FAULTS) {
			// A floating or shorted pin: stop the interrupts, Run() reports it.
			up_input_capture_set(_channel, Disabled, 0, nullptr, nullptr);
			_rate_failure.store(true);
			ScheduleNow();
		}

		return;
	}

	if (_rate_fault_counter > 0) {
		--_rate_fault_counter;
	}

	_last_edge_time = edge_time;

	const uint32_t write = _edge_write.load();
	_edges[write % EDGE_QUEUE_SIZE] = edge_time;
	_edge_write.store(write + 1);

	ScheduleNow();
}

void FormicStrobeCapture::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	if (_rate_failure.load()) {
		_rate_failure.store(false);
		_capturing = false;
		PX4_ERR("edge rate too high on channel %d, capture disabled", _channel + 1);
	}

	const uint32_t write = _edge_write.load();

	// The work queue fell behind by more than the queue: the oldest edges are gone.
	if (write - _edge_read > EDGE_QUEUE_SIZE) {
		const uint32_t lost = write - _edge_read - EDGE_QUEUE_SIZE;
		_dropped += lost;
		_seq += lost;
		_edge_read = write - EDGE_QUEUE_SIZE;
	}

	while (_edge_read != write) {
		camera_trigger_s strobe{};
		strobe.timestamp = _edges[_edge_read % EDGE_QUEUE_SIZE];
		strobe.seq = _seq++;
		strobe.feedback = true;
		_edge_read++;

		if (_last_publish_time != 0 && strobe.timestamp > _last_publish_time) {
			_last_interval_us = static_cast<uint32_t>(strobe.timestamp - _last_publish_time);
		}

		_last_publish_time = strobe.timestamp;
		_strobe_pub.publish(strobe);
	}
}

int FormicStrobeCapture::print_status()
{
	PX4_INFO("channel: %d, capturing: %s", _channel + 1, _capturing ? "yes" : "no");
	PX4_INFO("strobes: %" PRIu32 ", dropped: %" PRIu32 ", last interval: %" PRIu32 " us", _seq, _dropped,
		 _last_interval_us);

	if (_last_publish_time != 0) {
		PX4_INFO("last strobe: %" PRIu64 " us (%d ms ago)", _last_publish_time,
			 (int)(hrt_elapsed_time(&_last_publish_time) / 1000));

	} else {
		PX4_INFO("no strobe yet");
	}

	if (_capturing) {
		input_capture_stats_t stats{};

		if (up_input_capture_get_stats(_channel, &stats, false) == 0) {
			PX4_INFO("timer: edges %" PRIu32 ", overflows %" PRIu32 ", latency %" PRIu16, stats.edges,
				 stats.overflows, stats.latency);
		}
	}

	return 0;
}

int FormicStrobeCapture::task_spawn(int argc, char *argv[])
{
	FormicStrobeCapture *instance = new FormicStrobeCapture();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

int FormicStrobeCapture::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormicStrobeCapture::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Timestamps the tracker camera strobe with the timer capture of its own pin and
publishes every edge as `tracker_strobe`.

camera_capture keeps the ring cameras' strobe on its Camera_Capture pin and the
`camera_trigger` topic; this driver shares no pin, topic or parameter with it.

Requires FSTRB_EN=1 and one PWM channel set to Formic_Tracker_Strobe. No other
channel of that timer may drive a motor or servo.

### Examples
$ formic_strobe_capture start
$ formic_strobe_capture status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_strobe_capture", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int formic_strobe_capture_main(int argc, char *argv[])
{
	return FormicStrobeCapture::main(argc, argv);
}
