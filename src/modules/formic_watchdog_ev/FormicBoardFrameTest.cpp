#include <gtest/gtest.h>

#include <cmath>

#include "board_frame.hpp"

using formic_watchdog_ev::body_from_sensor;
using formic_watchdog_ev::sensor_to_body_orientation;
using matrix::Dcmf;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

namespace
{
// The level trim of the drone on 28 Oct: SENS_BOARD_X_OFF 1.2947, SENS_BOARD_Y_OFF -1.1572, in degrees.
constexpr float kTrimRollDeg = 1.2947f;
constexpr float kTrimPitchDeg = -1.1572f;
constexpr float kTolerance = 1e-5f;
constexpr float kRadPerDeg = static_cast<float>(M_PI / 180.0);

Dcmf trim()
{
	return body_from_sensor(Eulerf{kTrimRollDeg * kRadPerDeg, kTrimPitchDeg * kRadPerDeg, 0.f}, Dcmf{});
}

// Returns the rotation vector of the step from `a` to `b`, in the frame of `a`.
Vector3f relative_rotation(const Quatf &a, const Quatf &b)
{
	return matrix::AxisAnglef(Dcmf(a).T() * Dcmf(b)).axis() * matrix::AxisAnglef(Dcmf(a).T() * Dcmf(b)).angle();
}

void expect_equal(const Dcmf &a, const Dcmf &b)
{
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			EXPECT_NEAR(a(i, j), b(i, j), kTolerance);
		}
	}
}
}

// No trim and no board rotation leave the orientation as it is.
TEST(FormicBoardFrame, IdentityKeepsTheOrientation)
{
	Quatf q(Eulerf{0.3f, -0.2f, 1.1f});
	float a[4] = {q(0), q(1), q(2), q(3)};
	ASSERT_TRUE(sensor_to_body_orientation(a, Dcmf{}));

	for (int i = 0; i < 4; i++) {
		EXPECT_NEAR(a[i], q(i), kTolerance);
	}
}

// A sensor frame that sits at the trim attitude is a level, aligned body: the result is the identity.
TEST(FormicBoardFrame, ASensorAtTheTrimAttitudeGivesALevelBody)
{
	const Dcmf R = trim();
	Quatf q(R);
	float a[4] = {q(0), q(1), q(2), q(3)};
	ASSERT_TRUE(sensor_to_body_orientation(a, R));
	EXPECT_NEAR(std::fabs(a[0]), 1.f, kTolerance);
	EXPECT_NEAR(a[1], 0.f, kTolerance);
	EXPECT_NEAR(a[2], 0.f, kTolerance);
	EXPECT_NEAR(a[3], 0.f, kTolerance);
}

// With the sensor level, the body orientation is the transpose of the trim.
TEST(FormicBoardFrame, ALevelSensorGivesTheInverseTrim)
{
	const Dcmf R = trim();
	float a[4] = {1.f, 0.f, 0.f, 0.f};
	ASSERT_TRUE(sensor_to_body_orientation(a, R));
	expect_equal(Dcmf(Quatf(a)), R.T());
}

// A turn seen by the sensor is the same turn rotated into the body frame, so the rates of the body
// are R times the rates of the sensor. This is what the EKF2 and VIO rates were compared with.
TEST(FormicBoardFrame, BodyRatesAreTheRotatedSensorRates)
{
	const Dcmf R = trim();
	const Quatf s1(Eulerf{0.4f, -0.3f, 0.9f});
	const Quatf s2(Eulerf{0.5f, -0.1f, 1.2f});
	float b1[4] = {s1(0), s1(1), s1(2), s1(3)};
	float b2[4] = {s2(0), s2(1), s2(2), s2(3)};
	ASSERT_TRUE(sensor_to_body_orientation(b1, R));
	ASSERT_TRUE(sensor_to_body_orientation(b2, R));

	const Vector3f omega_sensor = relative_rotation(s1, s2);
	const Vector3f omega_body = relative_rotation(Quatf(b1), Quatf(b2));
	const Vector3f expected = R * omega_sensor;

	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(omega_body(i), expected(i), 1e-4f);
	}
}

// A board rotation of 90 degrees about z turns the heading by -90 degrees.
TEST(FormicBoardFrame, ABoardRotationTurnsTheHeading)
{
	const Dcmf R = body_from_sensor(Eulerf{}, Dcmf(Eulerf{0.f, 0.f, static_cast<float>(M_PI_2)}));
	float a[4] = {1.f, 0.f, 0.f, 0.f};
	ASSERT_TRUE(sensor_to_body_orientation(a, R));
	EXPECT_NEAR(Eulerf(Quatf(a)).psi(), -static_cast<float>(M_PI_2), kTolerance);
}

// The trim comes after the board rotation.
TEST(FormicBoardFrame, TheTrimIsAppliedAfterTheBoardRotation)
{
	const Dcmf board(Eulerf{0.f, 0.f, static_cast<float>(M_PI_2)});
	const Eulerf level{kTrimRollDeg * kRadPerDeg, 0.f, 0.f};
	expect_equal(body_from_sensor(level, board), Dcmf(level) * board);
}

// A quaternion that is not finite or not near unit length is left alone and reported.
TEST(FormicBoardFrame, ABadQuaternionIsLeftAlone)
{
	float nan_q[4] = {NAN, 0.f, 0.f, 0.f};
	EXPECT_FALSE(sensor_to_body_orientation(nan_q, trim()));
	EXPECT_TRUE(std::isnan(nan_q[0]));

	float zero_q[4] = {0.f, 0.f, 0.f, 0.f};
	EXPECT_FALSE(sensor_to_body_orientation(zero_q, trim()));
	EXPECT_EQ(zero_q[0], 0.f);
	EXPECT_EQ(zero_q[1], 0.f);
}

// The result is a unit quaternion.
TEST(FormicBoardFrame, TheResultIsAUnitQuaternion)
{
	const Quatf q(Eulerf{2.5f, 0.7f, -2.0f});
	float a[4] = {q(0), q(1), q(2), q(3)};
	ASSERT_TRUE(sensor_to_body_orientation(a, trim()));
	EXPECT_NEAR(Quatf(a).length(), 1.f, kTolerance);
}
