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
 * @file TurtleMapping.hpp
 *
 * Quad-X corner -> PX4 logical motor (1..4) -> local DShot output bit mapping,
 * and stick -> motor selection. No actuator calls.
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>

namespace turtle_mapping
{

enum Corner : uint8_t { FL = 0, FR = 1, RL = 2, RR = 3 };

enum class Error : uint8_t {
	NONE, UNCONFIGURED, MOTOR_ID, DUPLICATE_MOTOR, OUTPUT_COUNT,
	MISSING_OUTPUT, DUPLICATE_OUTPUT, EXTRA_FUNCTION, INACTIVE_OUTPUT
};

inline const char *error_name(Error e)
{
	switch (e) {
	case Error::NONE: return "OK";

	case Error::UNCONFIGURED: return "UNCONFIGURED";

	case Error::MOTOR_ID: return "MOTOR_ID";

	case Error::DUPLICATE_MOTOR: return "DUPLICATE_MOTOR";

	case Error::OUTPUT_COUNT: return "OUTPUT_COUNT";

	case Error::MISSING_OUTPUT: return "MISSING_OUTPUT";

	case Error::DUPLICATE_OUTPUT: return "DUPLICATE_OUTPUT";

	case Error::EXTRA_FUNCTION: return "EXTRA_FUNCTION";

	case Error::INACTIVE_OUTPUT: return "INACTIVE_OUTPUT";
	}

	return "UNKNOWN";
}

/** TRTL_FL/FR/RL/RR: PX4 logical motor number (1..4) at each corner, 0 = unset. */
struct Config { int32_t motor[4] {0, 0, 0, 0}; };

struct Resolved {
	uint32_t corner[4] {0, 0, 0, 0}; // one output bit per corner
	uint32_t all{0};
	Error error{Error::UNCONFIGURED};
	bool valid() const { return error == Error::NONE; }
};

inline Error validate(const Config &cfg)
{
	for (unsigned i = 0; i < 4; ++i) {
		if (cfg.motor[i] == 0) { return Error::UNCONFIGURED; }

		// Deliberately restricted to a single four-motor vehicle.
		if (cfg.motor[i] < 1 || cfg.motor[i] > 4) { return Error::MOTOR_ID; }

		for (unsigned j = 0; j < i; ++j) {
			if (cfg.motor[i] == cfg.motor[j]) { return Error::DUPLICATE_MOTOR; }
		}
	}

	return Error::NONE;
}

/**
 * Resolve corners to output bits of this DShot instance.
 * @param functions output function per slot (OutputFunction cast to int; Motor1 == 101)
 * @param active_outputs bitmask of outputs this driver actually drives
 */
inline Resolved resolve(const Config &cfg, const int32_t *functions, size_t count, uint32_t active_outputs)
{
	Resolved r;
	r.error = validate(cfg);

	if (!r.valid()) { return r; }

	if (!functions || count == 0 || count > 32) { r.error = Error::OUTPUT_COUNT; return r; }

	// Only disabled outputs or exactly Motor 1..4 may share this DShot instance.
	for (size_t slot = 0; slot < count; ++slot) {
		const int32_t fn = functions[slot];

		if (fn == 0) { continue; }

		unsigned corner = 4;

		for (unsigned c = 0; c < 4; ++c) {
			if (fn == 100 + cfg.motor[c]) { corner = c; break; }
		}

		if (corner == 4) { r.error = Error::EXTRA_FUNCTION; return r; }

		const uint32_t bit = uint32_t{1} << slot;

		if (!(active_outputs & bit)) { r.error = Error::INACTIVE_OUTPUT; return r; }

		if (r.corner[corner]) { r.error = Error::DUPLICATE_OUTPUT; return r; }

		r.corner[corner] = bit;
		r.all |= bit;
	}

	for (unsigned c = 0; c < 4; ++c) {
		if (!r.corner[c]) { r.error = Error::MISSING_OUTPUT; return r; }
	}

	r.error = Error::NONE;
	return r;
}

inline bool same(const Resolved &a, const Resolved &b)
{
	if (!a.valid() || !b.valid() || a.all != b.all) { return false; }

	for (unsigned i = 0; i < 4; ++i) { if (a.corner[i] != b.corner[i]) { return false; } }

	return true;
}

/**
 * Stick -> motor selection. Cardinal roll/pitch selects a side/end pair,
 * a combined deflection selects the single intersecting corner.
 * Convention: +roll selects the right side, +pitch selects the front motors.
 */
inline uint32_t select(const Resolved &map, float roll, float pitch, float deadband = 0.35f)
{
	if (!map.valid() || !std::isfinite(roll) || !std::isfinite(pitch)
	    || std::fabs(roll) > 1.05f || std::fabs(pitch) > 1.05f) { return 0; }

	if (!std::isfinite(deadband) || deadband < 0.05f || deadband > 0.6f) { return 0; }

	uint32_t mask = map.all;
	bool selected = false;

	if (roll > deadband) { mask &= map.corner[FR] | map.corner[RR]; selected = true; }

	else if (roll < -deadband) { mask &= map.corner[FL] | map.corner[RL]; selected = true; }

	if (pitch > deadband) { mask &= map.corner[FL] | map.corner[FR]; selected = true; }

	else if (pitch < -deadband) { mask &= map.corner[RL] | map.corner[RR]; selected = true; }

	return selected ? mask : 0;
}

/** 1-based output number of a single-bit mask, 0 if not a single bit. */
inline unsigned output_number(uint32_t single_bit)
{
	if (!single_bit || (single_bit & (single_bit - 1))) { return 0; }

	unsigned n = 1;

	while ((single_bit >>= 1) != 0) { ++n; }

	return n;
}

} // namespace turtle_mapping
