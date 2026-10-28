#include <gtest/gtest.h>

#include "sample_time.hpp"

using formic_watchdog_ev::resolve_sample_time;
using formic_watchdog_ev::StampSource;

namespace
{
// The sender's host clock in us, as seen on the bench (about 1.793e15 us).
constexpr uint64_t kSentUs = 1'793'185'039'834'956ULL;
// The PX4 clock when the pose arrives, in us.
constexpr uint64_t kPublishPx4Us = 13'800'000'000ULL;
constexpr uint64_t kNowUs = kPublishPx4Us + 4'000;
// The sample time on the VIO clock that the control repo's openvins2px4 sends, in us.
constexpr uint64_t kVioClockSampleUs = 14'197'223'000ULL;
constexpr int64_t kAgeMaxUs = 200'000;
constexpr int64_t kAgeValidUs = 5'000'000;
}

// A pose that is 81.5 ms old gets the publish time minus its age.
TEST(FormicSampleTime, UsesTheAgeOfTheSender)
{
	const auto t = resolve_sample_time(kSentUs, kSentUs - 81'500, kPublishPx4Us, kNowUs, kAgeMaxUs, kAgeValidUs);
	EXPECT_EQ(t.source, StampSource::SENDER_AGE);
	EXPECT_EQ(t.age_us, 81'500);
	EXPECT_EQ(t.time_us, kPublishPx4Us - 81'500);
}

// The bench maximum of 123.6 ms is used.
TEST(FormicSampleTime, TheLargestAgeOfTheBenchIsUsed)
{
	const auto t = resolve_sample_time(kSentUs, kSentUs - 123'600, kPublishPx4Us, kNowUs, kAgeMaxUs, kAgeValidUs);
	EXPECT_EQ(t.source, StampSource::SENDER_AGE);
}

// An age equal to the largest age is used, one us more is stale.
TEST(FormicSampleTime, TheLargestAgeIsInclusive)
{
	EXPECT_EQ(resolve_sample_time(kSentUs, kSentUs - kAgeMaxUs, kPublishPx4Us, kNowUs, kAgeMaxUs, kAgeValidUs).source,
		  StampSource::SENDER_AGE);
	EXPECT_EQ(resolve_sample_time(kSentUs, kSentUs - kAgeMaxUs - 1, kPublishPx4Us, kNowUs, kAgeMaxUs, kAgeValidUs).source,
		  StampSource::STALE);
}

// A pose 354 ms old, the stall of the first bench bag, is stale.
TEST(FormicSampleTime, AStallIsStale)
{
	const auto t = resolve_sample_time(kSentUs, kSentUs - 354'000, kPublishPx4Us, kNowUs, kAgeMaxUs, kAgeValidUs);
	EXPECT_EQ(t.source, StampSource::STALE);
	EXPECT_EQ(t.age_us, 354'000);
}

// The sample stamp of openvins2px4 is on the VIO clock: the "age" is years, so the poll time is used.
TEST(FormicSampleTime, ASenderOnAnotherClockGetsThePollTime)
{
	const auto t = resolve_sample_time(kSentUs, kVioClockSampleUs, kPublishPx4Us, kNowUs, kAgeMaxUs, kAgeValidUs);
	EXPECT_EQ(t.source, StampSource::POLL_TIME);
	EXPECT_EQ(t.time_us, kNowUs);
}

// A sample stamped after the publish time is not an age.
TEST(FormicSampleTime, ANegativeAgeGetsThePollTime)
{
	const auto t = resolve_sample_time(kSentUs, kSentUs + 1'000, kPublishPx4Us, kNowUs, kAgeMaxUs, kAgeValidUs);
	EXPECT_EQ(t.source, StampSource::POLL_TIME);
	EXPECT_EQ(t.time_us, kNowUs);
}

// A largest age of 0 turns the sender's sample time off: every pose gets the poll time.
TEST(FormicSampleTime, ALargestAgeOfZeroKeepsThePollTime)
{
	const auto t = resolve_sample_time(kSentUs, kSentUs - 81'500, kPublishPx4Us, kNowUs, 0, kAgeValidUs);
	EXPECT_EQ(t.source, StampSource::POLL_TIME);
	EXPECT_EQ(t.time_us, kNowUs);
}

// A publish time smaller than the age would wrap around: the poll time is used.
TEST(FormicSampleTime, APublishTimeBeforeTheAgeGetsThePollTime)
{
	const auto t = resolve_sample_time(kSentUs, kSentUs - 81'500, 50'000, kNowUs, kAgeMaxUs, kAgeValidUs);
	EXPECT_EQ(t.source, StampSource::POLL_TIME);
}

// The sample time is never later than now.
TEST(FormicSampleTime, TheSampleTimeIsNotInTheFuture)
{
	const auto t = resolve_sample_time(kSentUs, kSentUs - 1'000, kNowUs + 50'000, kNowUs, kAgeMaxUs, kAgeValidUs);
	EXPECT_EQ(t.source, StampSource::SENDER_AGE);
	EXPECT_EQ(t.time_us, kNowUs);
}

// Poses 50 ms apart on the sender's clock get sample times 50 ms apart, whatever the age.
TEST(FormicSampleTime, SampleTimesKeepTheSpacingOfTheSamples)
{
	const auto a = resolve_sample_time(kSentUs + 81'000, kSentUs, kPublishPx4Us + 81'000, kNowUs + 90'000, kAgeMaxUs, kAgeValidUs);
	const auto b = resolve_sample_time(kSentUs + 50'000 + 95'000, kSentUs + 50'000, kPublishPx4Us + 50'000 + 95'000,
					   kNowUs + 160'000, kAgeMaxUs, kAgeValidUs);
	EXPECT_EQ(b.time_us - a.time_us, 50'000u);
}
