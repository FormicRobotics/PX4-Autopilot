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
 * @file camera_timesync.hpp
 *
 * Strobe-based clock sync between the companion computer and the flight
 * controller.
 *
 * The camera strobe (FSTROBE) is wired to the flight controller's camera
 * capture input. camera_capture stamps every edge with the hrt clock and
 * publishes it as camera_trigger. The companion stamps the same frame with
 * its own clock (V4L2 start-of-frame time) and sends it as
 * camera_frame_stamp over uXRCE-DDS. The strobe and the frame are the
 * same physical event, so
 *
 *     offset = strobe_hrt_us - frame_companion_us
 *
 * is a direct measurement of the clock offset, free of the link latency
 * that limits the round-trip DDS timesync. The filtered offset goes out as
 * camera_timesync_status; the uXRCE-DDS client applies it while locked.
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/camera_frame_stamp.h>
#include <uORB/topics/camera_timesync_status.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/timesync_status.h>

using namespace time_literals;

class CameraTimesync : public ModuleBase<CameraTimesync>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CameraTimesync();
	~CameraTimesync() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	// Ring of recent strobe edges. 128 entries cover more than one second at 100 fps,
	// well above the readout plus DDS transport delay of a frame stamp.
	static constexpr unsigned STROBE_RING_SIZE = 128;
	static constexpr unsigned MAX_LOCK_SAMPLES = 60;
	static constexpr unsigned MAX_CONSECUTIVE_REJECTS = 20;
	static constexpr hrt_abstime STATUS_PERIOD = 500_ms;

	struct Strobe {
		hrt_abstime timestamp{0};
		uint32_t seq{0};
		bool used{false};
	};

	void Run() override;
	void parameters_update(bool force = false);
	void drain_strobes();
	void process_frame(const camera_frame_stamp_s &frame);
	bool coarse_offset(int64_t frame_us, int64_t &offset_us) const;
	uint32_t effective_window() const;
	void update_strobe_period(hrt_abstime interval);
	void check_counters(const camera_frame_stamp_s &frame, const Strobe &strobe);
	double estimate_at(hrt_abstime t) const;
	void lock_from_samples();
	void unlock(const char *reason);
	void publish_status();

	// Strobe ring, newest entry at (_strobe_head - 1).
	Strobe _strobes[STROBE_RING_SIZE] {};
	unsigned _strobe_head{0};
	unsigned _strobe_count{0};
	hrt_abstime _last_strobe_time{0};
	uint32_t _strobe_period_us{0};
	uint32_t _strobe_intervals[9] {};
	unsigned _strobe_interval_count{0};
	unsigned _short_intervals{0};

	// Estimate: offset and its rate, valid at _estimate_time (the strobe time of the last sample).
	uint8_t _state{camera_timesync_status_s::STATE_NO_TRIGGERS};
	bool _locked{false};
	double _estimated_offset{0.0};
	double _drift_us_per_s{0.0};
	hrt_abstime _estimate_time{0};
	double _mean_abs_residual{100.0};
	int64_t _lock_samples[MAX_LOCK_SAMPLES] {};
	unsigned _lock_sample_count{0};
	hrt_abstime _lock_last_strobe_time{0};
	unsigned _consecutive_rejects{0};
	hrt_abstime _last_accept_time{0};
	hrt_abstime _last_frame_time{0};
	hrt_abstime _last_status_time{0};
	int64_t _dds_offset{0};
	bool _dds_offset_valid{false};
	// Last matched pair, for the counter checks.
	bool _last_match_valid{false};
	hrt_abstime _last_match_strobe_time{0};
	uint32_t _last_match_strobe_seq{0};
	uint32_t _last_match_frame_counter{0};

	// Last match, for the status message.
	camera_timesync_status_s _status{};

	uORB::SubscriptionCallbackWorkItem _frame_stamp_sub{this, ORB_ID(camera_frame_stamp)};
	uORB::SubscriptionCallbackWorkItem _camera_trigger_sub{this, ORB_ID(camera_trigger)};
	// timesync_status is a multi-instance topic: the DDS client and MAVLink each publish one.
	uORB::Subscription _timesync_status_subs[2] {{ORB_ID(timesync_status), 0}, {ORB_ID(timesync_status), 1}};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Publication<camera_timesync_status_s> _status_pub{ORB_ID(camera_timesync_status)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::CAMTS_APPLY>) _param_apply,
		(ParamInt<px4::params::CAMTS_WIN_US>) _param_window_us,
		(ParamInt<px4::params::CAMTS_MAX_DEV>) _param_max_dev_us,
		(ParamFloat<px4::params::CAMTS_ALPHA>) _param_alpha,
		(ParamInt<px4::params::CAMTS_LOCK_N>) _param_lock_n,
		(ParamInt<px4::params::CAMTS_TIMEOUT>) _param_timeout_ms,
		(ParamInt<px4::params::CAMTS_DELAY_US>) _param_delay_us
	)
};
