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
 * @file TurtleHelpers.hpp
 *
 * Small pure helpers used by the turtle recovery controller: topic freshness,
 * attitude "verticality" from a quaternion, RC channel health/selection and
 * Quad-X geometry sanity checks. No PX4 dependencies, unit-testable on host.
 */

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <math.h> // NAN without the C++ <limits> header

namespace turtle
{

inline bool fresh(uint64_t now, uint64_t stamp, uint64_t limit)
{
	return stamp && now >= stamp && now - stamp < limit;
}

/**
 * Body Z axis projected on earth down, from a (w,x,y,z) attitude quaternion.
 * 1 = upright, 0 = side-on, -1 = fully inverted. NAN for an invalid quaternion.
 */
inline float vertical(const float q[4])
{
	const float n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];

	if (!std::isfinite(n) || n < 0.9f || n > 1.1f) { return NAN; }

	return 1.f - 2.f * (q[1] * q[1] + q[2] * q[2]) / n;
}

/**
 * True if the chosen RC channel (1-based) is already mapped to a critical
 * rc_channels function. Sharing with generic AUX1..AUX6 is allowed.
 */
inline bool rc_conflict(const int8_t *functions, size_t count, int channel)
{
	if (channel < 1 || channel > 18 || !functions) { return true; }

	for (size_t i = 0; i < count; ++i) {
		if (i >= 8 && i <= 13) { continue; } // rc_channels_s::FUNCTION_AUX_1..6

		if (functions[i] == channel - 1) { return true; }
	}

	return false;
}

/**
 * Check that control-allocator rotor geometry agrees with a Quad-X corner
 * assignment: c = 0 FL, 1 FR, 2 RL, 3 RR; thrust axis straight down.
 */
inline bool quad_corner(unsigned c, float x, float y, float ax, float ay, float az)
{
	if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az)) {
		return false;
	}

	const float norm = std::sqrt(ax * ax + ay * ay + az * az);

	if (!std::isfinite(norm) || norm < 0.000001f) { return false; }

	ax /= norm;
	ay /= norm;
	az /= norm;
	const bool front = c < 2, right = c == 1 || c == 3;
	return (front ? x > 0.0001f : x < -0.0001f) && (right ? y > 0.0001f : y < -0.0001f)
	       && std::fabs(ax) < 0.01f && std::fabs(ay) < 0.01f && az < -0.99f;
}

namespace rc
{
static constexpr int32_t min_channel = 1;
static constexpr int32_t max_channel = 18;
static constexpr int32_t default_channel = 9;
static constexpr uint64_t max_age_us = 300000;

/** RC input is usable: not lost, fresh primary timestamp, fresh last-valid if provided. */
inline bool healthy(uint64_t now, uint64_t timestamp, uint64_t last_valid, bool signal_lost)
{
	if (signal_lost || timestamp == 0 || now < timestamp || now - timestamp >= max_age_us) {
		return false;
	}

	return last_valid == 0 || (now >= last_valid && now - last_valid < max_age_us);
}

/** Normalised value of a 1-based channel, NAN if unavailable. */
inline float select(const float *values, uint8_t count, int32_t channel)
{
	if (!values || channel < min_channel || channel > max_channel || count < channel) {
		return NAN;
	}

	const float value = values[channel - 1];
	return std::isfinite(value) ? value : NAN;
}
} // namespace rc

} // namespace turtle
