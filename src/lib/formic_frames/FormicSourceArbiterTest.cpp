#include <gtest/gtest.h>

#include "source_arbiter.hpp"

using namespace formic_frames;

namespace
{
constexpr uint64_t kSecond = 1'000'000;
constexpr uint64_t kStep = 100'000;

GpsSample goodGps()
{
	GpsSample gps;
	gps.fresh = true;
	gps.fix_type = 3;
	gps.satellites = 14;
	gps.hdop = 0.8f;
	gps.eph = 0.9f;
	gps.speed_acc = 0.2f;
	gps.ekf_fusing = true;
	gps.ekf_xy_valid = true;
	return gps;
}

struct Rig {
	SourceArbiter arbiter;
	ArbiterInput in;
	ArbiterOutput out;

	Rig() { in.now_us = 10 * kSecond; }

	/// Runs the arbiter for `duration_us` with the inputs as they are.
	void run(uint64_t duration_us)
	{
		const uint64_t end = in.now_us + duration_us;

		while (in.now_us < end) {
			in.now_us += kStep;
			out = arbiter.update(in);
		}
	}

	void setVio(bool good)
	{
		in.vio_enter = good;
		in.vio_stay = good;
	}
};
}

// Every row of the decision table, for the four settings.
TEST(FormicSourceArbiter, DecisionTable)
{
	const SourcePref prefs[] = {SourcePref::AUTO_VIO_FIRST, SourcePref::AUTO_GPS_FIRST, SourcePref::VIO_ONLY, SourcePref::GPS_ONLY};
	// rows: GPS good, VIO good; columns: the four settings
	const Source expected[4][4] = {
		{Source::VIO, Source::GPS, Source::VIO, Source::GPS},     // yes yes
		{Source::GPS, Source::GPS, Source::NONE, Source::GPS},    // yes no
		{Source::VIO, Source::VIO, Source::VIO, Source::NONE},    // no  yes
		{Source::NONE, Source::NONE, Source::NONE, Source::NONE}, // no  no
	};

	for (int row = 0; row < 4; row++) {
		const bool gps_good = (row < 2);
		const bool vio_good = (row % 2 == 0);

		for (int col = 0; col < 4; col++) {
			EXPECT_EQ(pick_source(prefs[col], gps_good, vio_good), expected[row][col]) << "row " << row << " col " << col;

			// the same through the arbiter, from a cold start
			Rig rig;
			rig.in.pref = prefs[col];
			rig.in.gps = gps_good ? goodGps() : GpsSample{};
			rig.setVio(vio_good);
			rig.run(7 * kSecond);
			EXPECT_EQ(rig.out.source, expected[row][col]) << "arbiter row " << row << " col " << col;
			EXPECT_EQ(rig.out.gps_good, gps_good);
			EXPECT_EQ(rig.out.vio_good, vio_good);
		}
	}
}

// GPS turns good only after every check has passed for t_good.
TEST(FormicSourceArbiter, GpsGoodAfterTheGoodTime)
{
	Rig rig;
	rig.in.pref = SourcePref::GPS_ONLY;
	rig.in.gps = goodGps();

	rig.run(4 * kSecond);
	EXPECT_FALSE(rig.out.gps_good);
	EXPECT_EQ(rig.out.source, Source::NONE);
	EXPECT_EQ(rig.out.gps_fail, 0);

	rig.run(1500000);
	EXPECT_TRUE(rig.out.gps_good);
	EXPECT_EQ(rig.out.source, Source::GPS);
}

// A failed check restarts the good time.
TEST(FormicSourceArbiter, FailedCheckRestartsTheGoodTime)
{
	Rig rig;
	rig.in.pref = SourcePref::GPS_ONLY;
	rig.in.gps = goodGps();
	rig.run(4 * kSecond);

	rig.in.gps.satellites = 8;
	rig.run(kStep);
	EXPECT_EQ(rig.out.gps_fail, gps_fail::SAT);

	rig.in.gps.satellites = 14;
	rig.run(4 * kSecond);
	EXPECT_FALSE(rig.out.gps_good);
	rig.run(1500000);
	EXPECT_TRUE(rig.out.gps_good);
}

// Good GPS turns off after a check has failed for t_bad, and names the check.
TEST(FormicSourceArbiter, GpsOffAfterTheBadTime)
{
	Rig rig;
	rig.in.pref = SourcePref::GPS_ONLY;
	rig.in.gps = goodGps();
	rig.run(6 * kSecond);
	ASSERT_TRUE(rig.out.gps_good);

	rig.in.gps.eph = 3.5f;
	rig.run(500000);
	EXPECT_TRUE(rig.out.gps_good);
	EXPECT_EQ(rig.out.gps_fail, gps_fail::EPH);

	rig.run(700000);
	EXPECT_FALSE(rig.out.gps_good);
	EXPECT_EQ(rig.out.source, Source::NONE);
	EXPECT_EQ(rig.out.gps_fail, gps_fail::EPH);
}

// A short failure does not turn good GPS off.
TEST(FormicSourceArbiter, ShortFailureIsRiddenThrough)
{
	Rig rig;
	rig.in.pref = SourcePref::GPS_ONLY;
	rig.in.gps = goodGps();
	rig.run(6 * kSecond);

	rig.in.gps.speed_acc = 0.9f;
	rig.run(500000);
	rig.in.gps.speed_acc = 0.2f;
	rig.run(3 * kSecond);
	EXPECT_TRUE(rig.out.gps_good);
	EXPECT_EQ(rig.out.gps_fail, 0);
}

// To turn off, satellites and HDOP must pass the limit by the margin: no flicker at the limit.
TEST(FormicSourceArbiter, MarginOnSatellitesAndHdop)
{
	Rig rig;
	rig.in.pref = SourcePref::GPS_ONLY;
	rig.in.gps = goodGps();
	rig.run(6 * kSecond);
	ASSERT_TRUE(rig.out.gps_good);

	// under the limit of 10, inside the margin of 2
	rig.in.gps.satellites = 8;
	rig.in.gps.hdop = 1.9f;
	rig.run(5 * kSecond);
	EXPECT_TRUE(rig.out.gps_good);

	rig.in.gps.satellites = 7;
	rig.run(1200000);
	EXPECT_FALSE(rig.out.gps_good);
	EXPECT_NE(rig.out.gps_fail & gps_fail::SAT, 0);

	// not good: the limit itself applies again, so 9 satellites do not turn it on
	rig.in.gps.satellites = 9;
	rig.in.gps.hdop = 0.8f;
	rig.run(7 * kSecond);
	EXPECT_FALSE(rig.out.gps_good);

	rig.in.gps.satellites = 10;
	rig.run(6 * kSecond);
	EXPECT_TRUE(rig.out.gps_good);
}

// A lost fix, a lost message, jamming and spoofing turn GPS off at once.
TEST(FormicSourceArbiter, HardFailuresActAtOnce)
{
	for (int kind = 0; kind < 4; kind++) {
		Rig rig;
		rig.in.pref = SourcePref::AUTO_GPS_FIRST;
		rig.in.gps = goodGps();
		rig.setVio(true);
		rig.run(6 * kSecond);
		ASSERT_EQ(rig.out.source, Source::GPS);

		uint16_t bit = 0;

		switch (kind) {
		case 0: rig.in.gps.fix_type = 2; bit = gps_fail::FIX; break;

		case 1: rig.in.gps.fresh = false; bit = gps_fail::STALE; break;

		case 2: rig.in.gps.jammed = true; bit = gps_fail::JAM; break;

		default: rig.in.gps.spoofed = true; bit = gps_fail::SPOOF; break;
		}

		rig.run(kStep);
		EXPECT_FALSE(rig.out.gps_good) << "kind " << kind;
		EXPECT_EQ(rig.out.gps_fail & bit, bit) << "kind " << kind;
		// the other source takes over in the same cycle
		EXPECT_EQ(rig.out.source, Source::VIO) << "kind " << kind;
	}
}

// EKF2 must fuse GPS and hold a valid position.
TEST(FormicSourceArbiter, EkfMustAcceptGps)
{
	Rig rig;
	rig.in.pref = SourcePref::GPS_ONLY;
	rig.in.gps = goodGps();
	rig.in.gps.ekf_fusing = false;
	rig.run(7 * kSecond);
	EXPECT_FALSE(rig.out.gps_good);
	EXPECT_EQ(rig.out.gps_fail, gps_fail::EKF);

	rig.in.gps.ekf_fusing = true;
	rig.in.gps.ekf_xy_valid = false;
	rig.run(7 * kSecond);
	EXPECT_FALSE(rig.out.gps_good);
}

// Values that are not finite fail their check.
TEST(FormicSourceArbiter, NanFails)
{
	GpsSample gps = goodGps();
	gps.hdop = NAN;
	gps.eph = NAN;
	gps.speed_acc = NAN;
	const ArbiterLimits lim;
	EXPECT_EQ(gps_checks(gps, lim, false), gps_fail::HDOP | gps_fail::EPH | gps_fail::SACC);
}

// The flying source fails: the other one takes over at once, inside the dwell time too.
TEST(FormicSourceArbiter, FailoverIgnoresTheDwellTime)
{
	Rig rig;
	rig.in.pref = SourcePref::AUTO_VIO_FIRST;
	rig.in.gps = goodGps();
	rig.setVio(false);
	rig.run(6 * kSecond);
	ASSERT_EQ(rig.out.source, Source::GPS);

	rig.setVio(true);
	rig.run(4 * kSecond);
	ASSERT_EQ(rig.out.source, Source::VIO);

	rig.setVio(false);
	rig.run(kStep);
	EXPECT_EQ(rig.out.source, Source::GPS);
}

// The preferred source comes back: no switch back inside the dwell time.
TEST(FormicSourceArbiter, NoSwitchBackInsideTheDwellTime)
{
	Rig rig;
	rig.in.pref = SourcePref::AUTO_VIO_FIRST;
	rig.in.gps = goodGps();
	rig.setVio(true);
	rig.run(6 * kSecond);
	ASSERT_EQ(rig.out.source, Source::VIO);

	rig.setVio(false);
	rig.run(kStep);
	ASSERT_EQ(rig.out.source, Source::GPS);

	rig.setVio(true);
	rig.run(2500000);
	EXPECT_EQ(rig.out.source, Source::GPS);
	EXPECT_TRUE(rig.out.vio_good);

	rig.run(700000);
	EXPECT_EQ(rig.out.source, Source::VIO);
}

// The pilot's choice acts at once: the RC switch wins over the parameter.
TEST(FormicSourceArbiter, RcSwitchWinsAndActsAtOnce)
{
	Rig rig;
	rig.in.pref = SourcePref::AUTO_VIO_FIRST;
	rig.in.rc = SourceRc::AUTO;
	rig.in.gps = goodGps();
	rig.setVio(true);
	rig.run(6 * kSecond);
	ASSERT_EQ(rig.out.source, Source::VIO);
	EXPECT_EQ(rig.out.setting, SourcePref::AUTO_VIO_FIRST);

	rig.in.rc = SourceRc::GPS;
	rig.run(kStep);
	EXPECT_EQ(rig.out.source, Source::GPS);
	EXPECT_EQ(rig.out.setting, SourcePref::GPS_ONLY);

	rig.in.rc = SourceRc::VIO;
	rig.run(kStep);
	EXPECT_EQ(rig.out.source, Source::VIO);
	EXPECT_EQ(rig.out.setting, SourcePref::VIO_ONLY);

	// "only": the other source is never used
	rig.setVio(false);
	rig.run(2 * kSecond);
	EXPECT_EQ(rig.out.source, Source::NONE);

	// back to the parameter
	rig.in.rc = SourceRc::AUTO;
	rig.run(kStep);
	EXPECT_EQ(rig.out.source, Source::GPS);

	// a change of the parameter acts at once too
	rig.setVio(true);
	rig.in.rc = SourceRc::NONE;
	rig.in.pref = SourcePref::VIO_ONLY;
	rig.run(kStep);
	EXPECT_EQ(rig.out.source, Source::VIO);
}

// The VIO carries on with less than it needs to be switched to.
TEST(FormicSourceArbiter, VioStaysWithoutBeingOfferable)
{
	Rig rig;
	rig.in.pref = SourcePref::VIO_ONLY;
	rig.setVio(true);
	rig.run(kSecond);
	ASSERT_EQ(rig.out.source, Source::VIO);

	rig.in.vio_enter = false;
	rig.run(5 * kSecond);
	EXPECT_EQ(rig.out.source, Source::VIO);

	rig.in.vio_stay = false;
	rig.run(kStep);
	EXPECT_EQ(rig.out.source, Source::NONE);

	// and is not taken again until it may be entered
	rig.in.vio_stay = true;
	rig.run(5 * kSecond);
	EXPECT_EQ(rig.out.source, Source::NONE);
}

// Flying on the VIO, a GPS whose speed differs is kept out and never removes the VIO.
TEST(FormicSourceArbiter, SpeedMismatchKeepsGpsOut)
{
	Rig rig;
	rig.in.pref = SourcePref::AUTO_GPS_FIRST;
	rig.setVio(true);
	rig.run(kSecond);
	ASSERT_EQ(rig.out.source, Source::VIO);

	rig.in.gps = goodGps();
	rig.in.speeds_valid = true;
	rig.in.gps_speed = 3.f;
	rig.in.vio_speed = 1.5f;
	rig.run(10 * kSecond);
	EXPECT_EQ(rig.out.source, Source::VIO);
	EXPECT_FALSE(rig.out.gps_good);
	EXPECT_EQ(rig.out.gps_fail, gps_fail::SPEED);

	rig.in.vio_speed = 2.9f;
	rig.run(kStep);
	EXPECT_EQ(rig.out.source, Source::GPS);
}

// Flying on GPS, a VIO whose speed differs is kept out and never removes GPS.
TEST(FormicSourceArbiter, SpeedMismatchKeepsVioOut)
{
	Rig rig;
	rig.in.pref = SourcePref::AUTO_VIO_FIRST;
	rig.in.gps = goodGps();
	rig.run(6 * kSecond);
	ASSERT_EQ(rig.out.source, Source::GPS);

	// moving at 4 m/s: 0.9 m/s is under the fixed limit and over 20 %
	rig.setVio(true);
	rig.in.speeds_valid = true;
	rig.in.gps_speed = 4.f;
	rig.in.vio_speed = 3.1f;
	rig.run(10 * kSecond);
	EXPECT_EQ(rig.out.source, Source::GPS);
	EXPECT_TRUE(rig.out.gps_good);
	EXPECT_FALSE(rig.out.vio_good);
	EXPECT_TRUE(rig.out.vio_speed_blocked);

	rig.in.vio_speed = 3.8f;
	rig.run(kStep);
	EXPECT_EQ(rig.out.source, Source::VIO);
	EXPECT_FALSE(rig.out.vio_speed_blocked);
}

// In a hover the speeds agree whatever the scale.
TEST(FormicSourceArbiter, NoSpeedCheckWithoutValidSpeeds)
{
	Rig rig;
	rig.in.pref = SourcePref::AUTO_GPS_FIRST;
	rig.setVio(true);
	rig.in.gps = goodGps();
	rig.in.speeds_valid = false;
	rig.in.gps_speed = 5.f;
	rig.in.vio_speed = 0.f;
	rig.run(7 * kSecond);
	EXPECT_EQ(rig.out.source, Source::GPS);
}
