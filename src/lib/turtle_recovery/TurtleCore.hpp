/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *   Turtle recovery based on "PX4 Turtle Recovery" by Yisrael Ben-Shlomo
 *   (BENSHLOMO/px4-fpv-extensions, BSD-3-Clause), generalised for the
 *   Formic Robotics PX4 fork.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

/**
 * @file TurtleCore.hpp
 *
 * Hardware-independent Quad-X turtle (crash-flip) recovery controller.
 *
 * Sequence:  IDLE -> HOLD -> PRESTOP -> REVERSE (DShot 21) -> SETTLE -> DRIVE
 * Exit:      -> POSTSTOP -> RESTORE (DShot 20) -> COOLDOWN -> WAIT_OFF -> IDLE
 * Startup:   POSTSTOP -> RESTORE -> COOLDOWN -> WAIT_OFF once, so the ESCs
 *            are normalised before the first arm is allowed.
 *
 * DShot 20/21 are *relative* to the direction stored in the ESC and volatile,
 * so no per-motor direction state is kept here. RESTORE means the restore
 * commands were transmitted; the ESC never acknowledges them.
 *
 * The controller only produces Frames; it makes no actuator calls.
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "ArmingInterlock.hpp"

namespace turtle
{

enum class Phase : uint8_t { IDLE, HOLD, PRESTOP, REVERSE, SETTLE, DRIVE, POSTSTOP, RESTORE, COOLDOWN, WAIT_OFF, FAULT };
enum class Action : uint8_t { NORMAL, STOP, REVERSE, RESTORE, THROTTLE };
enum class Reason : uint8_t { NONE, SWITCH, RC, THROTTLE, UPRIGHT, SAFETY, ARMED, MAPPING, TIMEOUT, CALLBACK_GAP, CLOCK, COMMAND_TIMEOUT };

inline const char *phase_name(Phase p)
{
	switch (p) {
	case Phase::IDLE: return "IDLE";

	case Phase::HOLD: return "HOLD";

	case Phase::PRESTOP: return "PRESTOP";

	case Phase::REVERSE: return "REVERSE";

	case Phase::SETTLE: return "SETTLE";

	case Phase::DRIVE: return "DRIVE";

	case Phase::POSTSTOP: return "POSTSTOP";

	case Phase::RESTORE: return "RESTORE";

	case Phase::COOLDOWN: return "COOLDOWN";

	case Phase::WAIT_OFF: return "WAIT_OFF";

	case Phase::FAULT: return "FAULT";
	}

	return "UNKNOWN";
}

inline const char *reason_name(Reason r)
{
	switch (r) {
	case Reason::NONE: return "NONE";

	case Reason::SWITCH: return "SWITCH";

	case Reason::RC: return "RC";

	case Reason::THROTTLE: return "THROTTLE";

	case Reason::UPRIGHT: return "UPRIGHT";

	case Reason::SAFETY: return "SAFETY";

	case Reason::ARMED: return "ARMED";

	case Reason::MAPPING: return "MAPPING";

	case Reason::TIMEOUT: return "TIMEOUT";

	case Reason::CALLBACK_GAP: return "CALLBACK_GAP";

	case Reason::CLOCK: return "CLOCK";

	case Reason::COMMAND_TIMEOUT: return "COMMAND_TIMEOUT";
	}

	return "UNKNOWN";
}

/** Snapshot of the TRTL_* parameters plus the resolved output mask. */
struct Config {
	int32_t enabled{0}, esc{0}; // esc=1: operator confirms the ESC honours temporary DShot 20/21
	int32_t channel{9}, polarity{1};
	int32_t entry_deg{120}, exit_deg{35};
	int32_t start{850}, peak{1500}, ramp_ms{250}, drive_ms{1000};
	int32_t pre_ms{800}, post_ms{1000}, settle_ms{200}, cooldown_ms{300};
	int32_t deadband_pct{20}, proportional{1}, hold_ms{400};
	uint32_t outputs{0};

	bool valid() const
	{
		unsigned bits = 0;

		for (uint32_t n = outputs; n; n &= n - 1) { ++bits; }

		return (enabled == 0 || enabled == 1) && (esc == 0 || esc == 1) && channel >= 1 && channel <= 18
		       && (polarity == 1 || polarity == -1) && entry_deg >= 60 && entry_deg <= 175
		       && exit_deg >= 5 && exit_deg <= 80 && exit_deg + 10 <= entry_deg
		       && start >= 1 && start <= 1999 && peak >= start && peak <= 1999
		       && ramp_ms >= 0 && ramp_ms <= 1500 && drive_ms >= 100 && drive_ms <= 3000 && ramp_ms <= drive_ms
		       && pre_ms >= 100 && pre_ms <= 5000 && post_ms >= 200 && post_ms <= 5000
		       && settle_ms >= 100 && settle_ms <= 1000 && cooldown_ms >= 100 && cooldown_ms <= 2000
		       && deadband_pct >= 5 && deadband_pct <= 60 && (proportional == 0 || proportional == 1)
		       && hold_ms >= 100 && hold_ms <= 1000 && bits == 4;
	}

	bool same(const Config &b) const
	{
		return enabled == b.enabled && esc == b.esc && channel == b.channel && polarity == b.polarity
		       && entry_deg == b.entry_deg && exit_deg == b.exit_deg && start == b.start && peak == b.peak
		       && ramp_ms == b.ramp_ms && drive_ms == b.drive_ms && pre_ms == b.pre_ms && post_ms == b.post_ms
		       && settle_ms == b.settle_ms && cooldown_ms == b.cooldown_ms && deadband_pct == b.deadband_pct
		       && proportional == b.proportional && hold_ms == b.hold_ms && outputs == b.outputs;
	}
};

/** Gated inputs, evaluated by the driver every step. */
struct Input {
	bool disarmed{false}, hardware_ok{false}, mapping_ok{false}, rc_ok{false}, throttle_low{false};
	bool switch_on{false}, switch_off{false}, attitude_ok{false};
	float vertical{1.f};	// body Z dot earth down: 1 upright, 0 side-on, -1 inverted
	uint32_t selected{0};	// output bits selected by the sticks
	float strength{0.f};	// max |roll|,|pitch| in [0,1]
};

/** What the driver must put on the DShot bus this cycle. */
struct Frame {
	Action action{Action::NORMAL};
	uint32_t outputs{0};
	uint16_t level{0};
};

class Core
{
public:
	explicit Core(Interlock &lock) : _lock(lock) {}

	Phase phase{Phase::IDLE};
	Reason reason{Reason::NONE};
	uint32_t attempts{0}, restore_sequences_sent{0};
	uint32_t reverse_frames{0}, restore_frames{0}, throttle_frames{0};
	uint32_t last_drive_ms{0};

	bool configured() const { return _configured; }
	bool busy() const { return phase != Phase::IDLE; }
	bool owns_outputs() const { return _owned; }
	bool synced() const { return _synced; }
	bool enabled() const { return _configured && _cfg.enabled == 1 && _cfg.esc == 1; }
	const Config &config() const { return _cfg; }

	bool configure(const Config &cfg)
	{
		if (busy() || !cfg.valid()) { return false; }

		if (_configured && _cfg.same(cfg)) { return true; }

		const bool resync = !_configured || _cfg.outputs != cfg.outputs || _cfg.esc != cfg.esc
				    || _cfg.enabled != cfg.enabled;
		_cfg = cfg;
		_configured = true;
		_off_since = 0;
		_off_qualified = false;

		if (resync) { _synced = false; _lock.set_ready(false); }

		_entry_cos = std::cos(float(cfg.entry_deg) * 0.017453292519943295f);
		_exit_cos = std::cos(float(cfg.exit_deg) * 0.017453292519943295f);
		return true;
	}

	void invalidate()
	{
		if (!busy()) { _configured = false; _synced = false; _off_qualified = false; _lock.set_ready(false); }
	}

	void fault(Reason why) { reason = why; phase = Phase::FAULT; _lock.latch_fault(); _owned = true; }

	Frame step(uint64_t now, const Input &in)
	{
		if (phase == Phase::FAULT) { return stop(); }

		if (!_configured || !enabled()) { return {}; }

		const uint64_t dt = _last ? now - _last : 0;

		if (_last && now < _last) {
			if (_owned) { fault(Reason::CLOCK); return stop(); }

			_off_qualified = false;
			_off_since = 0;

		} else if (_last && dt > 100000 && drive_side()) {
			abort(now, Reason::CALLBACK_GAP);
		}

		_last = now;

		if (_owned && !in.mapping_ok) { fault(Reason::MAPPING); return stop(); }

		if (_owned && !in.disarmed) { abort(now, Reason::ARMED); }

		// Startup normalisation: wait, send temporary NORMAL, then require switch OFF.
		if (phase == Phase::IDLE && !_synced) {
			_lock.set_ready(false);

			if (!in.mapping_ok || !in.disarmed || !in.hardware_ok || !_lock.acquire_recovery()) { return {}; }

			_owned = true;
			_startup = true;
			_since = now;
			_transaction_start = now;
			phase = Phase::POSTSTOP;
		}

		if (phase == Phase::IDLE) {
			if (!in.disarmed || !in.hardware_ok || !in.mapping_ok || !in.rc_ok || !in.throttle_low) {
				_off_since = 0;
				_off_qualified = false;
				return {};
			}

			if (in.switch_off) {
				if (!_off_since) { _off_since = now; }

				if (now >= _off_since && now - _off_since >= release_us) { _off_qualified = true; }

				return {};
			}

			if (!in.switch_on) { _off_since = 0; return {}; }

			// Consume the OFF->ON edge even if the start attitude is wrong, so that
			// leaving the switch ON and later tilting cannot launch a recovery.
			const bool edge = _off_qualified;
			_off_qualified = false;
			_off_since = 0;

			if (!edge || !in.attitude_ok || in.vertical > _entry_cos || !_lock.acquire_recovery()) { return {}; }

			_owned = true;
			_startup = false;
			phase = Phase::HOLD;
			_since = now;
			_transaction_start = now;
			reason = Reason::NONE;
			_lock.set_ready(false);
		}

		if (drive_side()) {
			Reason why = Reason::NONE;

			if (!in.disarmed) { why = Reason::ARMED; }

			else if (!in.hardware_ok || !in.mapping_ok) { why = Reason::SAFETY; }

			else if (!in.rc_ok) { why = Reason::RC; }

			else if (!in.switch_on) { why = Reason::SWITCH; }

			else if (!in.throttle_low) { why = Reason::THROTTLE; }

			else if (!in.attitude_ok) { why = Reason::SAFETY; }

			else if ((phase == Phase::HOLD && in.vertical > _entry_cos) || in.vertical >= _exit_cos) { why = Reason::UPRIGHT; }

			if (why != Reason::NONE) { abort(now, why); }
		}

		// Fail closed on an incomplete restoration or a stalled callback schedule.
		if (_owned && phase != Phase::WAIT_OFF && now >= _transaction_start
		    && now - _transaction_start > transaction_budget_us()) {
			fault(Reason::COMMAND_TIMEOUT);
			return stop();
		}

		switch (phase) {
		case Phase::IDLE:
			return {};

		case Phase::HOLD:
			if (elapsed(now, ms(_cfg.hold_ms))) { phase = Phase::PRESTOP; _since = now; ++attempts; }

			return stop();

		case Phase::PRESTOP:
			if (!elapsed(now, ms(_cfg.pre_ms))) { return stop(); }

			begin_command(Phase::REVERSE, now);
			return command(now, true);

		case Phase::REVERSE:
			return command(now, true);

		case Phase::SETTLE:
			if (!elapsed(now, ms(_cfg.settle_ms))) { return stop(); }

			phase = Phase::DRIVE;
			_since = now;
			_drive_start = now;
			_selection_since = now;
			_selected = 0;
			return drive(now, in);

		case Phase::DRIVE:
			return drive(now, in);

		case Phase::POSTSTOP:
			// Reset the dwell whenever a stop/calibration/test condition prevents restoration.
			if (!in.disarmed || !in.hardware_ok) { _since = now; return stop(); }

			if (!elapsed(now, ms(_cfg.post_ms))) { return stop(); }

			begin_command(Phase::RESTORE, now);
			return command(now, false);

		case Phase::RESTORE:
			if (!in.disarmed || !in.hardware_ok) { phase = Phase::POSTSTOP; _since = now; return stop(); }

			return command(now, false);

		case Phase::COOLDOWN:
			if (!in.disarmed || !in.hardware_ok) { phase = Phase::POSTSTOP; _since = now; return stop(); }

			if (elapsed(now, ms(_cfg.cooldown_ms))) { phase = Phase::WAIT_OFF; _off_since = 0; }

			return stop();

		case Phase::WAIT_OFF:
			if (!in.disarmed || !in.hardware_ok || !in.mapping_ok || !in.rc_ok || !in.throttle_low || !in.switch_off) {
				_off_since = 0;
				return stop();
			}

			if (!_off_since) { _off_since = now; return stop(); }

			if (now >= _off_since && now - _off_since >= release_us) {
				_synced = true;
				_owned = false;
				_startup = false;
				phase = Phase::IDLE;
				_off_qualified = true;
				_lock.set_ready(true);
				_lock.release_recovery();
				return {};
			}

			return stop();

		case Phase::FAULT:
			return stop();
		}

		return stop();
	}

private:
	static constexpr uint64_t release_us = 300000, command_dwell_us = 50000, command_timeout_us = 2000000;
	static constexpr uint32_t command_min_frames = 24; // ESCs need >= 6 matching frames; margin for drops

	Interlock &_lock;
	Config _cfg{};
	bool _configured{false}, _synced{false}, _owned{false}, _startup{false}, _off_qualified{false};
	uint64_t _last{0}, _since{0}, _transaction_start{0}, _off_since{0}, _drive_start{0}, _selection_since{0};
	uint32_t _count{0}, _selected{0};
	float _entry_cos{-0.5f}, _exit_cos{0.81915f};

	static uint64_t ms(int32_t v) { return uint64_t(v) * 1000; }
	bool elapsed(uint64_t now, uint64_t duration) const { return now >= _since && now - _since >= duration; }
	static Frame stop() { return {Action::STOP, 0, 0}; }

	bool drive_side() const
	{
		return phase == Phase::HOLD || phase == Phase::PRESTOP || phase == Phase::REVERSE || phase == Phase::SETTLE
		       || phase == Phase::DRIVE;
	}

	void abort(uint64_t now, Reason why)
	{
		if (!drive_side()) { return; }

		if (phase == Phase::DRIVE) { last_drive_ms = uint32_t((now - _drive_start) / 1000); }

		reason = why;
		phase = Phase::POSTSTOP;
		_since = now;
		_selected = 0;
		_count = 0;
	}

	uint64_t transaction_budget_us() const
	{
		return ms(_cfg.hold_ms + _cfg.pre_ms + _cfg.settle_ms + _cfg.drive_ms + _cfg.post_ms + _cfg.cooldown_ms)
		       + 2 * command_timeout_us + 2000000;
	}

	void begin_command(Phase p, uint64_t now) { phase = p; _since = now; _count = 0; }

	Frame command(uint64_t now, bool reverse)
	{
		if (elapsed(now, command_timeout_us)) { fault(Reason::COMMAND_TIMEOUT); return stop(); }

		++_count;

		if (reverse) { ++reverse_frames; } else { ++restore_frames; }

		if (_count >= command_min_frames && elapsed(now, command_dwell_us)) {
			phase = reverse ? Phase::SETTLE : Phase::COOLDOWN;
			_since = now;

			if (!reverse) { ++restore_sequences_sent; }
		}

		return {reverse ? Action::REVERSE : Action::RESTORE, _cfg.outputs, 0};
	}

	Frame drive(uint64_t now, const Input &in)
	{
		if (elapsed(now, ms(_cfg.drive_ms))) {
			last_drive_ms = uint32_t((now - _drive_start) / 1000);
			abort(now, Reason::TIMEOUT);
			return stop();
		}

		const uint32_t selected = in.selected & _cfg.outputs;

		if (!selected || !std::isfinite(in.strength)) { _selected = 0; _selection_since = now; return stop(); }

		if (selected != _selected) { _selected = selected; _selection_since = now; }

		const float db = float(_cfg.deadband_pct) * 0.01f;
		float fraction = (in.strength - db) / (1.f - db);

		if (fraction < 0.f) { fraction = 0.f; }

		if (fraction > 1.f) { fraction = 1.f; }

		const uint16_t target = _cfg.proportional
					? uint16_t(_cfg.start + float(_cfg.peak - _cfg.start) * fraction)
					: uint16_t(_cfg.peak);
		const uint64_t ramp = ms(_cfg.ramp_ms), since = now - _selection_since;
		const uint16_t cap = (!ramp || since >= ramp)
				     ? uint16_t(_cfg.peak)
				     : uint16_t(_cfg.start + (uint64_t(_cfg.peak - _cfg.start) * since) / ramp);
		++throttle_frames;
		return {Action::THROTTLE, selected, target < cap ? target : cap};
	}
};

} // namespace turtle
