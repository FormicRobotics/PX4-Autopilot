#include <gtest/gtest.h>

#include <cmath>

#include "anchor.hpp"
#include "pose_history.hpp"

using formic_frames::Anchor;
using formic_frames::apply_tilt;
using formic_frames::PoseHistory;
using formic_frames::PoseSample;
using formic_frames::ramp_scale;
using formic_frames::Step;
using formic_frames::step_between;
using formic_frames::tilt_offset;
using formic_frames::yaw_of;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

namespace
{
constexpr float kTolerance = 1e-4f;
constexpr float kRadPerDeg = static_cast<float>(M_PI / 180.0);

float angle_between(const Quatf &a, const Quatf &b)
{
	Quatf dq = (a * b.inversed()).normalized();
	return 2.f * acosf(fminf(fabsf(dq(0)), 1.f));
}

float tilt_between(const Quatf &a, const Quatf &b)
{
	const Vector3f za = matrix::Dcmf(a) * Vector3f(0.f, 0.f, 1.f);
	const Vector3f zb = matrix::Dcmf(b) * Vector3f(0.f, 0.f, 1.f);
	return atan2f(za.cross(zb).norm(), za.dot(zb)); // accurate for small angles, unlike acos
}

void expect_near(const Vector3f &a, const Vector3f &b, float tol = kTolerance)
{
	for (int i = 0; i < 3; i++) {
		EXPECT_NEAR(a(i), b(i), tol);
	}
}

// The pose of the published estimate, of EKF2 and of the VIO instance at one time.
struct Pose {
	Quatf q;
	Vector3f p;
	Vector3f v;
};

Pose anchored(const Anchor &a, const Pose &x)
{
	return Pose{a.apply(x.q), a.applyPosition(x.p), a.applyVector(x.v)};
}
}

// A new anchor puts the instance onto the published pose: same heading and position at t_h.
TEST(FormicAnchor, ANewAnchorKeepsThePublishedPose)
{
	const Pose pub{Quatf(Eulerf{0.02f, -0.03f, 1.2f}), Vector3f(3.f, -2.f, -1.5f), Vector3f(0.5f, 0.1f, 0.f)};
	const Pose inst{Quatf(Eulerf{0.025f, -0.02f, -2.5f}), Vector3f(-40.f, 7.f, -1.4f), Vector3f(0.1f, 0.4f, 0.f)};

	const Anchor a = Anchor::between(pub.q, pub.p, inst.q, inst.p);
	const Pose out = anchored(a, inst);

	EXPECT_NEAR(matrix::wrap_pi(yaw_of(out.q) - yaw_of(pub.q)), 0.f, kTolerance);
	expect_near(out.p, pub.p);

	// with the full tilt ramp applied the attitude is the published one
	const Vector3f d_theta = tilt_offset(pub.q, out.q);
	EXPECT_NEAR(d_theta(2), 0.f, kTolerance);
	EXPECT_LT(angle_between(apply_tilt(d_theta, 1.f, out.q), pub.q), 1e-3f);
}

// The anchor only rotates about down: the tilt of the instance is kept.
TEST(FormicAnchor, TheAnchorKeepsTheTilt)
{
	const Quatf q(Eulerf{0.2f, -0.1f, 0.7f});
	const Anchor a{2.f, Vector3f(1.f, 2.f, 3.f)};
	// the angle from level is the same; only the direction of the lean turns with the heading
	EXPECT_NEAR(tilt_between(a.apply(q), Quatf{}), tilt_between(q, Quatf{}), kTolerance);
	EXPECT_NEAR(Eulerf(a.apply(q)).phi(), 0.2f, kTolerance);
	EXPECT_NEAR(Eulerf(a.apply(q)).theta(), -0.1f, kTolerance);
}

// The ramp starts at the published attitude, moves without jumps and is within 5 % after 3 time constants.
TEST(FormicAnchor, TheTiltRampIsContinuousAndDecays)
{
	const float tau = 0.5f;
	const Quatf q_pub(Eulerf{1.5f * kRadPerDeg, -1.f * kRadPerDeg, 0.3f});
	const Quatf q_inst(Eulerf{0.f, 0.f, 0.3f});
	const Vector3f d_theta = tilt_offset(q_pub, q_inst);
	const float tilt0 = tilt_between(q_pub, q_inst);

	EXPECT_LT(angle_between(apply_tilt(d_theta, ramp_scale(0.f, tau), q_inst), q_pub), 1e-3f);

	Quatf prev = apply_tilt(d_theta, ramp_scale(0.f, tau), q_inst);

	for (int i = 1; i <= 300; i++) {
		const float t = i * 0.005f;
		const Quatf q = apply_tilt(d_theta, ramp_scale(t, tau), q_inst);
		// no step: each 5 ms moves by much less than the whole offset
		EXPECT_LT(angle_between(q, prev), 0.02f * tilt0);
		prev = q;
	}

	const Quatf q_3tau = apply_tilt(d_theta, ramp_scale(3.f * tau, tau), q_inst);
	EXPECT_LT(tilt_between(q_3tau, q_inst), 0.05f * d_theta.norm());
	EXPECT_EQ(ramp_scale(1.f, 0.f), 0.f);
}

// Position -> Altitude -> Position within one VIO segment publishes the old VIO frame again.
TEST(FormicAnchor, ARoundTripInOneSegmentReturnsToTheOldFrame)
{
	// EKF2 frame and VIO frame differ by 90 deg and 50 m
	const Anchor vio_from_ekf{static_cast<float>(M_PI_2), Vector3f(50.f, 0.f, 0.f)};

	// t1: EKF2 flies; switch to the VIO
	const Pose ekf_t1{Quatf(Eulerf{0.f, 0.f, 0.2f}), Vector3f(1.f, 1.f, -2.f), Vector3f(0.f, 0.f, 0.f)};
	const Pose vio_t1 = anchored(vio_from_ekf, ekf_t1);
	const Anchor a0{}; // EKF2 published as it is at boot
	const Pose pub_t1 = anchored(a0, ekf_t1);
	const Anchor a1 = Anchor::between(pub_t1.q, pub_t1.p, vio_t1.q, vio_t1.p);

	// t2: Position on the VIO; switch to EKF2, which is re-anchored onto the published pose
	const Pose true_t2{Quatf(Eulerf{0.f, 0.f, 0.5f}), Vector3f(10.f, 4.f, -2.f), Vector3f(1.f, 0.f, 0.f)};
	const Pose vio_t2 = anchored(vio_from_ekf, true_t2);
	const Pose ekf_t2 = true_t2; // no drift yet
	const Pose pub_t2 = anchored(a1, vio_t2);
	const Anchor a0_new = Anchor::between(pub_t2.q, pub_t2.p, ekf_t2.q, ekf_t2.p);
	expect_near(anchored(a0_new, ekf_t2).p, pub_t2.p);

	// t3: Altitude on EKF2, which drifts by 3 m and 0.1 rad; switch back to the VIO, same segment
	const Pose true_t3{Quatf(Eulerf{0.f, 0.f, 0.9f}), Vector3f(20.f, -6.f, -2.5f), Vector3f(0.f, 1.f, 0.f)};
	const Pose vio_t3 = anchored(vio_from_ekf, true_t3);
	const Pose ekf_t3{Quatf(Eulerf{0.f, 0.f, 1.0f}), true_t3.p + Vector3f(3.f, 0.f, 0.f), true_t3.v};
	const Pose pub_alt_t3 = anchored(a0_new, ekf_t3);
	const Pose pub_pos_t3 = anchored(a1, vio_t3); // the old anchor is kept

	// the published pose is where it would be had the VIO flown all along
	const Pose pub_if_vio = anchored(a1, vio_t3);
	expect_near(pub_pos_t3.p, pub_if_vio.p);
	expect_near(pub_pos_t3.p, anchored(a0, true_t3).p, 1e-3f); // EKF2's boot frame, which a1 continues

	// and the step back removes EKF2's drift
	const Step step = step_between(pub_alt_t3.q, pub_alt_t3.p, pub_alt_t3.v, pub_pos_t3.q, pub_pos_t3.p, pub_pos_t3.v);
	expect_near(step.delta_p, Vector3f(-3.f, 0.f, 0.f), 1e-3f);
	EXPECT_NEAR(step.delta_heading, -0.1f, 1e-3f);
}

// The published reset deltas add up to the jump.
TEST(FormicAnchor, TheResetDeltasAddUpToTheStep)
{
	const Quatf q_from(Eulerf{0.1f, 0.05f, -2.9f});
	const Quatf q_to(Eulerf{0.08f, 0.06f, 3.0f});
	const Vector3f p_from(1.f, 2.f, 3.f), p_to(-4.f, 5.f, 2.5f);
	const Vector3f v_from(1.5f, 0.f, 0.2f), v_to(0.f, 0.1f, -0.1f);

	const Step s = step_between(q_from, p_from, v_from, q_to, p_to, v_to);
	expect_near(p_from + s.delta_p, p_to);
	expect_near(v_from + s.delta_v, v_to);
	EXPECT_NEAR(matrix::wrap_pi(yaw_of(q_from) + s.delta_heading - yaw_of(q_to)), 0.f, kTolerance);
	EXPECT_LT(angle_between(s.delta_q * q_from, q_to), 1e-3f);
	// the heading change goes the short way across +-pi
	EXPECT_LT(fabsf(s.delta_heading), 0.5f);
}

// A reset delta of an instance turns with the anchor.
TEST(FormicAnchor, ResetDeltasTurnWithTheAnchor)
{
	const Anchor a{static_cast<float>(M_PI_2), Vector3f(5.f, 5.f, 0.f)};
	const matrix::Vector2f d = a.applyVector(matrix::Vector2f(1.f, 0.f));
	EXPECT_NEAR(d(0), 0.f, kTolerance);
	EXPECT_NEAR(d(1), 1.f, kTolerance);
	const matrix::Vector2f back = a.unapplyVector(d);
	EXPECT_NEAR(back(0), 1.f, kTolerance);
	EXPECT_NEAR(back(1), 0.f, kTolerance);

	const Quatf q_old(Eulerf{0.1f, 0.f, 0.3f});
	const Quatf dq(Eulerf{0.f, 0.f, 0.2f});
	const Quatf q_new = dq * q_old;
	EXPECT_LT(angle_between(a.applyDeltaQ(dq) * a.apply(q_old), a.apply(q_new)), 1e-3f);
}

// The history is read back between, before and after its samples.
TEST(FormicPoseHistory, InterpolatesAndExtrapolates)
{
	PoseHistory<8> h;
	PoseSample s;
	EXPECT_FALSE(h.sampleAt(0, s));

	for (int i = 0; i < 12; i++) {
		PoseSample x;
		x.time_us = 1000000 + i * 10000;
		x.q = Quatf(Eulerf{0.f, 0.f, 0.01f * i});
		x.p = Vector3f(0.01f * i, 0.f, 0.f);
		x.v = Vector3f(1.f, 0.f, 0.f);
		h.push(x);
	}

	// between samples 10 and 11
	ASSERT_TRUE(h.sampleAt(1105000, s));
	EXPECT_NEAR(s.p(0), 0.105f, 1e-5f);
	EXPECT_NEAR(yaw_of(s.q), 0.105f, 1e-4f);

	// after the newest: moved on by its velocity
	ASSERT_TRUE(h.sampleAt(1120000, s));
	EXPECT_NEAR(s.p(0), 0.12f, 1e-5f);
	EXPECT_FALSE(h.sampleAt(1200000, s));

	// older than the oldest kept (sample 4): the oldest
	ASSERT_TRUE(h.sampleAt(1000000, s));
	EXPECT_NEAR(s.p(0), 0.04f, 1e-5f);

	// a sample that is not newer is dropped
	PoseSample old;
	old.time_us = 1050000;
	h.push(old);
	EXPECT_EQ(h.newest().time_us, 1110000u);
}
