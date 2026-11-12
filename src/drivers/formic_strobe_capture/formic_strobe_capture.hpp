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

/**
 * @file formic_strobe_capture.hpp
 *
 * Timestamps the tracker camera strobe on its own capture pin.
 *
 * camera_capture stamps one pin only (the ring cameras' strobe) and publishes
 * it as camera_trigger. The tracker camera runs at another rate, so its strobe
 * gets its own pin (output function Formic_Tracker_Strobe), its own parameters
 * (FSTRB_*) and its own topic (tracker_strobe). Nothing here reads CAM_CAP_*
 * or publishes camera_trigger.
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_pwm_output.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/camera_trigger.h>

using namespace time_literals;

class FormicStrobeCapture : public ModuleBase<FormicStrobeCapture>, public px4::ScheduledWorkItem
{
public:
	FormicStrobeCapture();
	~FormicStrobeCapture() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	// Output function number of the pin, see mixer_module/output_functions.yaml.
	static constexpr int32_t OUTPUT_FUNCTION = 2090;

	// Edges closer than this to the previous one are glitches (5 kHz).
	static constexpr hrt_abstime MIN_EDGE_INTERVAL = 200_us;
	static constexpr uint8_t MAX_RATE_FAULTS = 100;

	// Edges waiting for the work queue. 8 slots are 130 ms of strobes at 60 Hz.
	static constexpr uint32_t EDGE_QUEUE_SIZE = 8;

	void Run() override;

	static void capture_trampoline(void *context, uint32_t chan_index, hrt_abstime edge_time, uint32_t edge_state,
				       uint32_t overflow);
	void capture_callback(hrt_abstime edge_time);

	int _channel{-1};
	bool _capturing{false};

	// Written by the capture interrupt, read by Run(): one writer, one reader.
	hrt_abstime _edges[EDGE_QUEUE_SIZE] {};
	px4::atomic<uint32_t> _edge_write{0};
	uint32_t _edge_read{0};
	hrt_abstime _last_edge_time{0};
	uint8_t _rate_fault_counter{0};
	px4::atomic<bool> _rate_failure{false};

	uint32_t _seq{0};
	uint32_t _dropped{0};
	hrt_abstime _last_publish_time{0};
	uint32_t _last_interval_us{0};

	uORB::Publication<camera_trigger_s> _strobe_pub{ORB_ID(tracker_strobe)};
};
