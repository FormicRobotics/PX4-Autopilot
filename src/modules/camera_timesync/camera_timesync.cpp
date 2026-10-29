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

#include "camera_timesync.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <mathlib/mathlib.h>

#include <stdlib.h>

namespace
{

// Sorts `values[0..count)` in place (insertion sort, count is at most 60).
void sort_i64(int64_t *values, unsigned count)
{
	for (unsigned i = 1; i < count; i++) {
		int64_t v = values[i];
		unsigned j = i;

		while (j > 0 && values[j - 1] > v) {
			values[j] = values[j - 1];
			j--;
		}

		values[j] = v;
	}
}

uint32_t median_u32(const uint32_t *values, unsigned count)
{
	uint32_t copy[16];
	count = (count > 16) ? 16 : count;

	for (unsigned i = 0; i < count; i++) {
		copy[i] = values[i];
	}

	for (unsigned i = 1; i < count; i++) {
		uint32_t v = copy[i];
		unsigned j = i;

		while (j > 0 && copy[j - 1] > v) {
			copy[j] = copy[j - 1];
			j--;
		}

		copy[j] = v;
	}

	return copy[count / 2];
}

} // namespace

CameraTimesync::CameraTimesync() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	parameters_update(true);
}

CameraTimesync::~CameraTimesync()
{
	ScheduleClear();
}

bool CameraTimesync::init()
{
	if (!_frame_stamp_sub.registerCallback()) {
		PX4_ERR("camera_frame_stamp callback registration failed");
		return false;
	}

	if (!_camera_trigger_sub.registerCallback()) {
		PX4_ERR("camera_trigger callback registration failed");
		return false;
	}

	// Periodic pass for the timeouts and the heartbeat status.
	ScheduleOnInterval(100_ms);
	return true;
}

void CameraTimesync::parameters_update(bool force)
{
	if (force || _parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void CameraTimesync::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_frame_stamp_sub.unregisterCallback();
		_camera_trigger_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	// The DDS round-trip estimate is the coarse hint before the first lock and the
	// comparison value afterwards.
	for (auto &sub : _timesync_status_subs) {
		timesync_status_s ts;

		if (sub.update(&ts) && ts.source_protocol == timesync_status_s::SOURCE_PROTOCOL_DDS) {
			_dds_offset = ts.estimated_offset;
			// The filter starts from zero; it is usable once a raw sample agrees with it.
			_dds_offset_valid = llabs(ts.observed_offset - ts.estimated_offset) < 5000;
		}
	}

	drain_strobes();

	camera_frame_stamp_s frame;

	while (_frame_stamp_sub.update(&frame)) {
		process_frame(frame);
	}

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime timeout = static_cast<hrt_abstime>(_param_timeout_ms.get()) * 1000;

	if (_strobe_count == 0 || (now - _last_strobe_time) > timeout) {
		if (_locked) {
			unlock("no strobes");
		}

		_state = camera_timesync_status_s::STATE_NO_TRIGGERS;

	} else if (_last_frame_time == 0 || (now - _last_frame_time) > timeout) {
		if (_locked) {
			unlock("no frame stamps");
		}

		_state = camera_timesync_status_s::STATE_NO_FRAMES;

	} else if (_locked && (now - _last_accept_time) > timeout) {
		unlock("no accepted sample");
		_state = camera_timesync_status_s::STATE_STALE;
	}

	if ((now - _last_status_time) > STATUS_PERIOD) {
		publish_status();
	}
}

void CameraTimesync::drain_strobes()
{
	camera_trigger_s trigger;

	while (_camera_trigger_sub.update(&trigger)) {
		if (_last_strobe_time != 0 && trigger.timestamp > _last_strobe_time) {
			update_strobe_period(trigger.timestamp - _last_strobe_time);
		}

		_last_strobe_time = trigger.timestamp;

		Strobe &slot = _strobes[_strobe_head];
		slot.timestamp = trigger.timestamp;
		slot.seq = trigger.seq;
		slot.used = false;
		_strobe_head = (_strobe_head + 1) % STROBE_RING_SIZE;

		if (_strobe_count < STROBE_RING_SIZE) {
			_strobe_count++;
		}
	}
}

void CameraTimesync::update_strobe_period(hrt_abstime interval)
{
	// Clamp to 1 s so a pause between sessions does not pollute the median.
	if (interval > 1_s) {
		return;
	}

	// Bounce on the strobe line shows as extra edges a fraction of a period after the real one.
	// They are not intervals of the camera; only a long run of them means the rate really changed.
	if (_strobe_period_us > 0 && interval < _strobe_period_us / 2) {
		if (++_short_intervals < 20) {
			return;
		}
	}

	_short_intervals = 0;

	constexpr unsigned n = sizeof(_strobe_intervals) / sizeof(_strobe_intervals[0]);
	_strobe_intervals[_strobe_interval_count % n] = static_cast<uint32_t>(interval);
	_strobe_interval_count++;
	_strobe_period_us = median_u32(_strobe_intervals, (_strobe_interval_count < n) ? _strobe_interval_count : n);
}

uint32_t CameraTimesync::effective_window() const
{
	if (_param_window_us.get() > 0) {
		return static_cast<uint32_t>(_param_window_us.get());
	}

	if (_strobe_period_us > 0) {
		return (_strobe_period_us * 45) / 100;
	}

	return 5000;
}

bool CameraTimesync::coarse_offset(int64_t frame_us, int64_t &offset_us) const
{
	if (_locked) {
		// Carry the estimate to where this frame's strobe should be, so a fast-slewing
		// companion clock (ms per second on the bench) does not show up as match error.
		const int64_t expected_hrt = frame_us + static_cast<int64_t>(_estimated_offset);
		offset_us = static_cast<int64_t>(estimate_at(expected_hrt > 0 ? static_cast<hrt_abstime>(expected_hrt) : 0));
		return true;
	}

	if (_dds_offset_valid) {
		offset_us = _dds_offset;
		return true;
	}

	return false;
}

void CameraTimesync::process_frame(const camera_frame_stamp_s &frame)
{
	const hrt_abstime now = hrt_absolute_time();
	_last_frame_time = now;

	if (_strobe_count == 0) {
		_state = camera_timesync_status_s::STATE_NO_TRIGGERS;
		return;
	}

	const int64_t frame_us = static_cast<int64_t>(frame.frame_realtime_ns / 1000ULL);
	int64_t hint = 0;

	if (!coarse_offset(frame_us, hint)) {
		_state = camera_timesync_status_s::STATE_WAITING_DDS;
		return;
	}

	const int64_t predicted = frame_us + hint;

	// Nearest unused strobe to the predicted hrt time of the frame.
	int best = -1;
	int64_t best_err = INT64_MAX;

	for (unsigned i = 0; i < _strobe_count; i++) {
		const Strobe &s = _strobes[i];

		if (s.used) {
			continue;
		}

		const int64_t err = static_cast<int64_t>(s.timestamp) - predicted;

		if (llabs(err) < llabs(best_err)) {
			best_err = err;
			best = static_cast<int>(i);
		}
	}

	const uint32_t window = effective_window();
	_status.match_window = window;
	_status.match_error = (best >= 0) ? best_err : 0;

	if (best < 0 || llabs(best_err) > static_cast<int64_t>(window)) {
		_status.unmatched_count++;

		if (!_locked) {
			_state = camera_timesync_status_s::STATE_ACQUIRING;
		}

		return;
	}

	Strobe &strobe = _strobes[best];
	strobe.used = true;

	const int64_t observed = static_cast<int64_t>(strobe.timestamp) - frame_us + _param_delay_us.get();

	_status.frame_counter = frame.frame_counter;
	_status.driver_sequence = frame.driver_sequence;
	_status.strobe_seq = strobe.seq;
	_status.frame_realtime_ns = frame.frame_realtime_ns;
	_status.strobe_timestamp = strobe.timestamp;
	_status.observed_offset = observed;
	_status.strobe_to_receive = static_cast<uint32_t>(now - strobe.timestamp);
	// frame.timestamp was converted to hrt by the DDS client with whatever offset it had.
	_status.transport_latency = (now > frame.timestamp) ? static_cast<uint32_t>(now - frame.timestamp) : 0;

	if (!_locked) {
		_state = camera_timesync_status_s::STATE_ACQUIRING;

		if (_lock_sample_count < MAX_LOCK_SAMPLES) {
			_lock_samples[_lock_sample_count++] = observed;
			_lock_last_strobe_time = strobe.timestamp;
		}

		const unsigned needed = static_cast<unsigned>(math::constrain(_param_lock_n.get(), (int32_t)2, (int32_t)MAX_LOCK_SAMPLES));

		if (_lock_sample_count >= needed) {
			lock_from_samples();
		}

	} else {
		check_counters(frame, strobe);

		// Offset and drift (alpha-beta filter) at the strobe time of this sample.
		double dt_s = 0.0;

		if (strobe.timestamp > _estimate_time) {
			dt_s = static_cast<double>(strobe.timestamp - _estimate_time) * 1e-6;
			_estimated_offset += _drift_us_per_s * dt_s;
			_estimate_time = strobe.timestamp;
		}

		const double residual = static_cast<double>(observed) - _estimated_offset;

		// A sample far outside the recent residuals is a spurious edge the frame matched
		// (bounce on the strobe line). The floor lets a small step of the companion clock be
		// absorbed by the filter; CAMTS_MAX_DEV stays the ceiling that drops the lock.
		const double gate = math::min(static_cast<double>(_param_max_dev_us.get()),
					      math::max(300.0, 8.0 * _mean_abs_residual));

		if (fabs(residual) > gate) {
			_status.rejected_count++;
			_consecutive_rejects++;

			if (_consecutive_rejects > MAX_CONSECUTIVE_REJECTS) {
				unlock("offset jumped");
			}

			publish_status();
			return;
		}

		_consecutive_rejects = 0;
		_mean_abs_residual += 0.05 * (fabs(residual) - _mean_abs_residual);

		const double alpha = static_cast<double>(math::constrain(_param_alpha.get(), 0.01f, 1.f));
		const double beta = alpha * alpha / (2.0 - alpha);
		_estimated_offset += alpha * residual;

		if (dt_s > 0.0) {
			_drift_us_per_s = math::constrain(_drift_us_per_s + beta * residual / dt_s, -2000.0, 2000.0);
		}
	}

	_status.matched_count++;
	_last_accept_time = now;
	publish_status();
}

// Both counters are judged against the clock: the strobe period says how many frames passed
// between two matched strobes. The companion skips its counter over frames its driver dropped,
// so a frame step off that count is a frame lost without a stamp gap. A strobe seq step off it
// is an edge camera_capture counted with no strobe behind it (bounce on the line) or a missed
// edge; neither disturbs the offset, the match goes by time, but both are worth knowing.
void CameraTimesync::check_counters(const camera_frame_stamp_s &frame, const Strobe &strobe)
{
	if (_last_match_valid && frame.frame_counter < _last_match_frame_counter) {
		PX4_INFO("companion frame counter restarted (%lu -> %lu)",
			 (unsigned long)_last_match_frame_counter, (unsigned long)frame.frame_counter);
		_last_match_valid = false;
	}

	if (_last_match_valid && _strobe_period_us > 0 && strobe.timestamp > _last_match_strobe_time) {
		const int64_t elapsed = static_cast<int64_t>(strobe.timestamp - _last_match_strobe_time);
		const int64_t periods = (elapsed + _strobe_period_us / 2) / _strobe_period_us;
		const int64_t frame_step = static_cast<int64_t>(frame.frame_counter)
					   - static_cast<int64_t>(_last_match_frame_counter);
		const int64_t seq_step = static_cast<int64_t>(strobe.seq) - static_cast<int64_t>(_last_match_strobe_seq);

		if (frame_step != periods) {
			_status.counter_mismatch_count++;
		}

		if (seq_step != periods) {
			_status.strobe_seq_error_count++;
		}
	}

	_last_match_strobe_time = strobe.timestamp;
	_last_match_strobe_seq = strobe.seq;
	_last_match_frame_counter = frame.frame_counter;
	_last_match_valid = true;
}

void CameraTimesync::lock_from_samples()
{
	int64_t sorted[MAX_LOCK_SAMPLES];

	for (unsigned i = 0; i < _lock_sample_count; i++) {
		sorted[i] = _lock_samples[i];
	}

	sort_i64(sorted, _lock_sample_count);

	const int64_t spread = sorted[_lock_sample_count - 1] - sorted[0];

	if (spread > 2 * static_cast<int64_t>(_param_max_dev_us.get())) {
		// The samples do not agree: keep the newest half and collect more.
		const unsigned keep = _lock_sample_count / 2;

		for (unsigned i = 0; i < keep; i++) {
			_lock_samples[i] = _lock_samples[_lock_sample_count - keep + i];
		}

		_lock_sample_count = keep;
		return;
	}

	_estimated_offset = static_cast<double>(sorted[_lock_sample_count / 2]);
	_estimate_time = _lock_last_strobe_time;
	_drift_us_per_s = 0.0;
	_mean_abs_residual = 100.0;
	_locked = true;
	_state = camera_timesync_status_s::STATE_LOCKED;
	_consecutive_rejects = 0;
	_last_match_valid = false;
	_lock_sample_count = 0;

	PX4_INFO("locked: offset %lld us (dds %lld us, diff %lld us) from %u samples",
		 (long long)_estimated_offset, (long long)_dds_offset,
		 (long long)(static_cast<int64_t>(_estimated_offset) - _dds_offset), (unsigned)_param_lock_n.get());
}

void CameraTimesync::unlock(const char *reason)
{
	if (_locked) {
		PX4_WARN("lock dropped: %s", reason);
	}

	_locked = false;
	_lock_sample_count = 0;
	_consecutive_rejects = 0;
	_last_match_valid = false;
}

double CameraTimesync::estimate_at(hrt_abstime t) const
{
	if (!_locked || t <= _estimate_time) {
		return _estimated_offset;
	}

	return _estimated_offset + _drift_us_per_s * static_cast<double>(t - _estimate_time) * 1e-6;
}

void CameraTimesync::publish_status()
{
	_status.timestamp = hrt_absolute_time();
	_status.state = _state;
	_status.apply = _locked && (_param_apply.get() != 0);
	// The estimate is carried from the last strobe sample (~a frame readout plus the link ago) to now.
	_status.estimated_offset = static_cast<int64_t>(estimate_at(_status.timestamp));
	_status.drift_ppm = static_cast<float>(_drift_us_per_s);
	_status.dds_offset = _dds_offset;
	_status.strobe_period = _strobe_period_us;
	_status_pub.publish(_status);
	_last_status_time = _status.timestamp;
}

int CameraTimesync::print_status()
{
	static const char *state_names[] = {"NO_TRIGGERS", "NO_FRAMES", "WAITING_DDS", "ACQUIRING", "LOCKED", "STALE"};
	const char *state = (_state < 6) ? state_names[_state] : "?";

	PX4_INFO("state: %s, apply: %s", state, (_locked && _param_apply.get()) ? "yes" : "no");
	PX4_INFO("offset: camera %lld us, dds %lld us, camera - dds %lld us, drift %.1f ppm",
		 (long long)_estimated_offset, (long long)_dds_offset,
		 (long long)(static_cast<int64_t>(_estimated_offset) - _dds_offset), _drift_us_per_s);
	PX4_INFO("last: observed %lld us, match error %lld us, window %lu us, strobe period %lu us",
		 (long long)_status.observed_offset, (long long)_status.match_error,
		 (unsigned long)_status.match_window, (unsigned long)_strobe_period_us);
	PX4_INFO("latency: strobe->receive %lu us, transport %lu us",
		 (unsigned long)_status.strobe_to_receive, (unsigned long)_status.transport_latency);
	PX4_INFO("counts: matched %lu, rejected %lu, unmatched %lu, counter mismatch %lu, strobe seq errors %lu, strobes %u",
		 (unsigned long)_status.matched_count, (unsigned long)_status.rejected_count,
		 (unsigned long)_status.unmatched_count, (unsigned long)_status.counter_mismatch_count,
		 (unsigned long)_status.strobe_seq_error_count, _strobe_count);
	return 0;
}

int CameraTimesync::task_spawn(int argc, char *argv[])
{
	CameraTimesync *instance = new CameraTimesync();

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

int CameraTimesync::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CameraTimesync::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Strobe-based clock sync between the companion computer and the flight controller.

Matches each `camera_frame_stamp` from the companion with the `camera_trigger`
strobe edge that camera_capture stamped, and estimates the clock offset from the
pairs. The result is published as `camera_timesync_status`; while locked and
CAMTS_APPLY=1 the uXRCE-DDS client uses it in place of its round-trip estimate.

Requires CAM_CAP_FBACK=1, one PWM channel set to Camera_Capture, and the camera
strobe wired to it.

### Examples
$ camera_timesync start
$ camera_timesync status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("camera_timesync", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int camera_timesync_main(int argc, char *argv[])
{
	return CameraTimesync::main(argc, argv);
}
