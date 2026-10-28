/****************************************************************************
 * The sample time of a pose from formic_odom, on the PX4 clock.
 ****************************************************************************/

#pragma once

#include <cstdint>

namespace formic_watchdog_ev
{

/// Where the sample time of a pose comes from.
enum class StampSource : uint8_t {
	/// The publish time of the pose minus its age, both from the sender.
	SENDER_AGE,
	/// The time the module polled the pose. The stamps of the sender are not an age.
	POLL_TIME,
	/// The pose is older than the largest age. It is not used.
	STALE,
};

/// A sample time and what it was made from.
struct SampleTime {
	/// Sample time on the PX4 clock, in us. The poll time unless `source` is SENDER_AGE.
	uint64_t time_us;
	StampSource source;
	/// Age of the pose when the sender published it, in us, as computed. Negative or huge when
	/// the sender's stamps are on different clocks.
	int64_t age_us;
};

/**
 * Returns the sample time of a pose.
 *
 * The sender stamps the pose twice on its host clock: `sent_us` (odometry.timestamp) when it
 * publishes it and `sample_us` (odometry.timestamp_sample) for the time the pose is at. Their
 * difference is the age of the pose when it was published. `publish_px4_us` is the top-level
 * timestamp of formic_odom, which the DDS client has mapped onto the PX4 clock. The same
 * instant as `sent_us`, so `publish_px4_us - age` is the sample time on the PX4 clock.
 *
 * @param age_max_us   largest age that is used. 0 or less: the poll time is always used.
 * @param age_valid_us largest age taken as an age. A larger or a negative one means the two
 *                     stamps are on different clocks (the control repo's openvins2px4 stamps
 *                     the sample on the VIO clock): the poll time is used.
 * @param now_us       the PX4 clock now, the poll time
 */
inline SampleTime resolve_sample_time(uint64_t sent_us, uint64_t sample_us, uint64_t publish_px4_us,
				      uint64_t now_us, int64_t age_max_us, int64_t age_valid_us)
{
	const int64_t age_us = static_cast<int64_t>(sent_us) - static_cast<int64_t>(sample_us);
	SampleTime out{now_us, StampSource::POLL_TIME, age_us};

	if (age_max_us <= 0 || age_us < 0 || age_us > age_valid_us
	    || publish_px4_us <= static_cast<uint64_t>(age_us)) {
		return out;
	}

	if (age_us > age_max_us) {
		out.source = StampSource::STALE;
		return out;
	}

	out.source = StampSource::SENDER_AGE;
	out.time_us = publish_px4_us - static_cast<uint64_t>(age_us);

	if (out.time_us > now_us) {
		out.time_us = now_us;
	}

	return out;
}

} // namespace formic_watchdog_ev
