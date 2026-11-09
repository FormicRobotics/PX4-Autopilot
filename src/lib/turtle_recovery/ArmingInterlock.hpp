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
 * @file ArmingInterlock.hpp
 *
 * Software ownership interlock between normal flight (Commander arming) and
 * turtle recovery (DShot driver driving reversed motors while disarmed).
 * Shared within the monolithic PX4 firmware. This is NOT an ESC
 * acknowledgement that direction was actually restored.
 */

#pragma once

#include <stdint.h>

// GCC/Clang atomic builtins are used so the header does not depend on <atomic>
// (not available on all NuttX toolchain configurations).
static_assert(__atomic_always_lock_free(sizeof(uint32_t), nullptr),
	      "turtle recovery requires lock-free 32-bit atomics");
static_assert(__atomic_always_lock_free(sizeof(bool), nullptr),
	      "turtle recovery requires lock-free boolean atomics");

namespace turtle
{

enum class Owner : uint32_t { NONE, FLIGHT, RECOVERY, FAULT };

class Interlock
{
public:
	Interlock() = default;
	Interlock(const Interlock &) = delete;
	Interlock &operator=(const Interlock &) = delete;

	bool acquire_recovery()
	{
		uint32_t expected = uint32_t(Owner::NONE);
		return __atomic_compare_exchange_n(&_owner, &expected, uint32_t(Owner::RECOVERY), false,
						   __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	}

	bool acquire_flight(bool require_ready)
	{
		if (require_ready && !__atomic_load_n(&_ready, __ATOMIC_SEQ_CST)) { return false; }

		uint32_t expected = uint32_t(Owner::NONE);

		if (!__atomic_compare_exchange_n(&_owner, &expected, uint32_t(Owner::FLIGHT), false,
						 __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST)) {
			return false;
		}

		if (require_ready && !__atomic_load_n(&_ready, __ATOMIC_SEQ_CST)) {
			release_flight();
			return false;
		}

		return true;
	}

	void release_flight()
	{
		uint32_t expected = uint32_t(Owner::FLIGHT);
		__atomic_compare_exchange_n(&_owner, &expected, uint32_t(Owner::NONE), false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	}

	void release_recovery()
	{
		uint32_t expected = uint32_t(Owner::RECOVERY);
		__atomic_compare_exchange_n(&_owner, &expected, uint32_t(Owner::NONE), false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
	}

	void set_ready(bool ready) { __atomic_store_n(&_ready, ready, __ATOMIC_SEQ_CST); }
	bool ready() const { return __atomic_load_n(&_ready, __ATOMIC_SEQ_CST); }
	Owner owner() const { return Owner(__atomic_load_n(&_owner, __ATOMIC_SEQ_CST)); }
	bool recovery_owned() const { return owner() == Owner::RECOVERY || owner() == Owner::FAULT; }

	void latch_fault()
	{
		__atomic_store_n(&_ready, false, __ATOMIC_SEQ_CST);
		__atomic_store_n(&_owner, uint32_t(Owner::FAULT), __ATOMIC_SEQ_CST);
	}

private:
	alignas(4) uint32_t _owner{uint32_t(Owner::NONE)};
	bool _ready{false};
};

// A C++ inline function-local static has exactly one instance across
// translation units, so Commander and the DShot driver share this object.
inline Interlock &interlock()
{
	static Interlock value;
	return value;
}

/**
 * RAII guard used by Commander::arm(). Reserves FLIGHT ownership for the
 * duration of the arm transition. If the transition does not end in the armed
 * state, ownership is released again.
 */
class ArmGuard
{
public:
	ArmGuard(const uint8_t &state, uint8_t armed_value, bool require_ready)
		: _state(state), _armed(armed_value), _already(state == armed_value),
		  _reserved(!_already && interlock().acquire_flight(require_ready)) {}

	~ArmGuard()
	{
		if (_reserved && _state != _armed) { interlock().release_flight(); }
	}

	bool allowed() const { return _already || _reserved; }

	ArmGuard(const ArmGuard &) = delete;
	ArmGuard &operator=(const ArmGuard &) = delete;

private:
	const uint8_t &_state;
	uint8_t _armed;
	bool _already;
	bool _reserved;
};

/**
 * RAII guard used by Commander::disarm(). Releases FLIGHT ownership once the
 * vehicle is no longer armed.
 */
class DisarmGuard
{
public:
	DisarmGuard(const uint8_t &state, uint8_t armed_value) : _state(state), _armed(armed_value) {}

	~DisarmGuard()
	{
		if (_state != _armed) { interlock().release_flight(); }
	}

private:
	const uint8_t &_state;
	uint8_t _armed;
};

} // namespace turtle
