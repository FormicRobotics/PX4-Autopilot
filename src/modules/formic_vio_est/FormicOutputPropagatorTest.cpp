#include <gtest/gtest.h>

#include <cmath>
#include <random>

#include "OutputPropagator.hpp"
#include "VioSegment.hpp"

using formic_vio_est::ImuDelta;
using formic_vio_est::OutputPropagator;
using formic_vio_est::OutputState;
using formic_vio_est::PropagatorResult;
using formic_vio_est::SegmentAction;
using formic_vio_est::TrackingError;
using formic_vio_est::VioSegment;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

namespace
{
constexpr float kG = 9.80665f;
constexpr uint64_t kImuPeriodUs = 5000;   // 200 Hz, IMU_INTEG_RATE default
constexpr uint64_t kVioPeriodUs = 33333;  // 30 Hz
constexpr uint64_t kStartUs = 10000000;

using Propagator = OutputPropagator<160>;

// A truth the test can sample at any time.
struct Truth {
	virtual ~Truth() = default;
	virtual Vector3f p(double t) const = 0;
	virtual Vector3f v(double t) const = 0;
	virtual Vector3f a(double t) const = 0;
	virtual Quatf q(double t) const = 0;
	virtual Vector3f rate(double t) const = 0; // body frame
};

struct ConstantVelocity : Truth {
	Vector3f v0{1.f, 0.5f, -0.2f};
	Quatf q0{Eulerf{0.f, 0.f, 0.4f}};
	Vector3f p(double t) const override { return v0 * static_cast<float>(t); }
	Vector3f v(double) const override { return v0; }
	Vector3f a(double) const override { return {}; }
	Quatf q(double) const override { return q0; }
	Vector3f rate(double) const override { return {}; }
};

// Circles at 2 m/s while turning at the same rate, level.
struct Circle : Truth {
	float r{3.f};
	float w{2.f / 3.f};
	float angle(double t) const { return w * static_cast<float>(t); }
	Vector3f p(double t) const override { return Vector3f(r * cosf(angle(t)), r * sinf(angle(t)), -2.f); }
	Vector3f v(double t) const override { return Vector3f(-r * w * sinf(angle(t)), r * w * cosf(angle(t)), 0.f); }
	Vector3f a(double t) const override { return Vector3f(-r * w * w * cosf(angle(t)), -r * w * w * sinf(angle(t)), 0.f); }
	Quatf q(double t) const override { return Quatf(Eulerf{0.f, 0.f, angle(t)}); }
	Vector3f rate(double) const override { return Vector3f(0.f, 0.f, w); }
};

// The IMU sample ending at `t_us` for the truth.
ImuDelta imu_at(const Truth &truth, uint64_t t_us, const Vector3f &accel_bias = {})
{
	const double t = t_us * 1e-6;
	const float dt = kImuPeriodUs * 1e-6f;
	const double tm = t - 0.5 * static_cast<double>(dt);
	const matrix::Dcmf R(truth.q(tm));
	const Vector3f specific_force = R.T() * (truth.a(tm) - Vector3f(0.f, 0.f, kG));

	ImuDelta d;
	d.time_us = t_us;
	d.dt = dt;
	d.delta_ang = truth.rate(tm) * dt;
	d.delta_vel = (specific_force + accel_bias) * dt;
	return d;
}

struct FlightResult {
	float pos_err{0.f};
	float vel_err{0.f};
	float att_err{0.f};
};

/**
 * Flies `seconds` with VIO poses `delay_min`..`delay_max` ms late. The output starts `p_offset`
 * and `v_offset` away from the truth.
 */
FlightResult fly(const Truth &truth, double seconds, uint32_t delay_min_us, uint32_t delay_max_us,
	const Vector3f &p_offset = {}, const Vector3f &v_offset = {}, const Vector3f &accel_bias = {})
{
	Propagator prop;
	std::mt19937 rng(42);
	std::uniform_int_distribution<uint32_t> delay(delay_min_us, delay_max_us);

	struct Pending {
		uint64_t sample_us;
		uint64_t arrival_us;
	};

	Pending queue[16];
	int n_pending = 0;
	uint64_t next_vio_us = kStartUs;

	const uint64_t end_us = kStartUs + static_cast<uint64_t>(seconds * 1e6);

	for (uint64_t t = kStartUs; t <= end_us; t += kImuPeriodUs) {
		prop.update(imu_at(truth, t, accel_bias));

		while (next_vio_us <= t) {
			queue[n_pending++] = Pending{next_vio_us, next_vio_us + delay(rng)};
			next_vio_us += kVioPeriodUs;
		}

		// poses whose arrival time has come, in arrival order
		for (int i = 0; i < n_pending;) {
			if (queue[i].arrival_us <= t) {
				const double ts = queue[i].sample_us * 1e-6;

				if (!prop.initialised()) {
					prop.initialise(queue[i].sample_us, truth.q(ts), truth.v(ts) + v_offset, truth.p(ts) + p_offset);

				} else {
					prop.correct(queue[i].sample_us, truth.q(ts), truth.v(ts), true, truth.p(ts));
				}

				queue[i] = queue[--n_pending];

			} else {
				i++;
			}
		}
	}

	const OutputState &o = prop.output();
	const double t = o.time_us * 1e-6;
	FlightResult r;
	r.pos_err = (o.p - truth.p(t)).norm();
	r.vel_err = (o.v - truth.v(t)).norm();
	const Quatf dq = (o.q * truth.q(t).inversed()).normalized();
	r.att_err = 2.f * acosf(fminf(fabsf(dq(0)), 1.f));
	return r;
}
}

// A constant velocity with poses 50-125 ms late and jittered: the output now is within 1 cm and 1 cm/s.
TEST(FormicOutputPropagator, ConstantVelocityWithLatePoses)
{
	const ConstantVelocity truth;
	const FlightResult r = fly(truth, 10.0, 50000, 125000, Vector3f(0.4f, -0.3f, 0.1f), Vector3f(0.3f, 0.f, -0.1f));
	EXPECT_LT(r.pos_err, 0.01f);
	EXPECT_LT(r.vel_err, 0.01f);
	EXPECT_LT(r.att_err, 1e-3f);
}

// A turning, accelerating flight: the output now follows the truth, not the late poses.
TEST(FormicOutputPropagator, CircleWithLatePoses)
{
	const Circle truth;
	const FlightResult r = fly(truth, 10.0, 70000, 125000);
	EXPECT_LT(r.pos_err, 0.02f);
	EXPECT_LT(r.vel_err, 0.02f);
	EXPECT_LT(r.att_err, 2e-3f);
}

// An accelerometer bias leaves a small velocity error, not a drift.
TEST(FormicOutputPropagator, AccelerometerBiasDoesNotDrift)
{
	const ConstantVelocity truth;
	const FlightResult r = fly(truth, 20.0, 50000, 125000, {}, {}, Vector3f(0.05f, -0.05f, 0.f));
	EXPECT_LT(r.pos_err, 0.05f);
	EXPECT_LT(r.vel_err, 0.05f);
}

// A pose is placed by its time stamp: inside the history, too old, or in the future.
TEST(FormicOutputPropagator, PoseTimeChecks)
{
	const ConstantVelocity truth;
	Propagator prop;

	EXPECT_EQ(prop.correct(kStartUs, truth.q(0), truth.v(0), true, truth.p(0)), PropagatorResult::NOT_INITIALISED);

	for (uint64_t t = kStartUs; t < kStartUs + 2000000; t += kImuPeriodUs) {
		prop.update(imu_at(truth, t));
	}

	const uint64_t newest = prop.newestImuTime();
	const uint64_t t_s = newest - 100000;
	const double ts = t_s * 1e-6;
	ASSERT_EQ(prop.initialise(t_s, truth.q(ts), truth.v(ts), truth.p(ts)), PropagatorResult::ACCEPTED);

	// the output now is the pose carried 100 ms on
	const double tn = newest * 1e-6;
	EXPECT_LT((prop.output().p - truth.p(tn)).norm(), 1e-3f);

	TrackingError err;
	EXPECT_EQ(prop.correct(t_s + 20000, truth.q(ts), truth.v(ts), true, truth.p(ts + 0.02), &err), PropagatorResult::ACCEPTED);
	EXPECT_LT(err.pos, 1e-3f);

	// before the start of the output: too old; after the newest IMU sample: in the future
	EXPECT_EQ(prop.correct(t_s - 50000, truth.q(ts), truth.v(ts), true, truth.p(ts)), PropagatorResult::TOO_OLD);
	EXPECT_EQ(prop.correct(newest + 20000, truth.q(ts), truth.v(ts), true, truth.p(ts)), PropagatorResult::IN_FUTURE);
	EXPECT_EQ(prop.correct(newest + 8000, truth.q(tn), truth.v(tn), true, truth.p(tn)), PropagatorResult::ACCEPTED);

	// older than the whole IMU history (160 samples = 0.8 s)
	Propagator prop2;

	for (uint64_t t = kStartUs; t < kStartUs + 2000000; t += kImuPeriodUs) {
		prop2.update(imu_at(truth, t));
	}

	EXPECT_EQ(prop2.initialise(newest - 900000, truth.q(0), truth.v(0), truth.p(0)), PropagatorResult::TOO_OLD);
}

// A position error is removed with the position time constant, without overshoot.
TEST(FormicOutputPropagator, ErrorDecaysWithTheTimeConstant)
{
	const ConstantVelocity truth;
	Propagator prop;
	prop.setTimeConstants(0.1f, 0.25f, 0.25f);

	for (uint64_t t = kStartUs; t < kStartUs + 500000; t += kImuPeriodUs) {
		prop.update(imu_at(truth, t));
	}

	uint64_t t = prop.newestImuTime();
	double ts = t * 1e-6;
	ASSERT_EQ(prop.initialise(t, truth.q(ts), truth.v(ts), truth.p(ts) + Vector3f(1.f, 0.f, 0.f)), PropagatorResult::ACCEPTED);
	ASSERT_EQ(prop.correct(t, truth.q(ts), truth.v(ts), true, truth.p(ts)), PropagatorResult::ACCEPTED);

	// one time constant with no new pose: about 1 / e of the error is left
	for (int i = 0; i < 50; i++) {
		t += kImuPeriodUs;
		prop.update(imu_at(truth, t));
	}

	const float left = (prop.output().p - truth.p(t * 1e-6)).norm();
	EXPECT_NEAR(left, expf(-1.f), 0.02f);

	// a long gap removes the error, never more
	for (int i = 0; i < 400; i++) {
		t += kImuPeriodUs;
		prop.update(imu_at(truth, t));
		EXPECT_GT(prop.output().p(0) - truth.p(t * 1e-6)(0), -1e-3f);
	}
}

// A first pose starts a segment; a restart after a short gap is anchored; a long gap starts a new segment.
TEST(FormicVioSegment, EpochsAndGaps)
{
	VioSegment seg;
	const uint64_t gap_max = 500000;

	EXPECT_EQ(seg.classify(1, kStartUs, gap_max), SegmentAction::NEW_SEGMENT);
	seg.startSegment(1, kStartUs);
	EXPECT_EQ(seg.id(), 1u);

	EXPECT_EQ(seg.classify(1, kStartUs + 33000, gap_max), SegmentAction::CONTINUE);
	seg.used(kStartUs + 33000);

	// a restart 400 ms later: same segment, anchored onto the output
	const uint64_t t2 = kStartUs + 433000;
	EXPECT_EQ(seg.classify(2, t2, gap_max), SegmentAction::NEW_EPOCH);

	const Quatf q_out(Eulerf{0.f, 0.f, 1.f});
	const Vector3f p_out(5.f, 1.f, -2.f);
	const Quatf q_new(Eulerf{0.f, 0.f, -0.5f});
	const Vector3f p_new(0.f, 0.f, 0.f);
	seg.anchorEpoch(2, t2, q_out, p_out, q_new, p_new);
	EXPECT_EQ(seg.id(), 1u);
	EXPECT_EQ(seg.epochChanges(), 1u);

	// the new epoch's pose lands on the output
	const Vector3f p_mapped = seg.epochAnchor().applyPosition(p_new);
	EXPECT_LT((p_mapped - p_out).norm(), 1e-4f);
	EXPECT_NEAR(formic_frames::yaw_of(seg.epochAnchor().apply(q_new)), 1.f, 1e-4f);

	// a pose of the new epoch 1 m further on is 1 m further on in the segment frame, turned with it
	const Vector3f moved = seg.epochAnchor().applyPosition(p_new + Vector3f(1.f, 0.f, 0.f));
	EXPECT_NEAR((moved - p_out).norm(), 1.f, 1e-4f);

	// a gap longer than FORMIC_VIO_GAP_MAX: a new segment, in the frame of the new epoch
	EXPECT_EQ(seg.classify(2, t2 + 600000, gap_max), SegmentAction::NEW_SEGMENT);
	seg.startSegment(2, t2 + 600000);
	EXPECT_EQ(seg.id(), 2u);
	EXPECT_EQ(seg.epochChanges(), 0u);
	EXPECT_LT(seg.epochAnchor().applyPosition(p_new).norm(), 1e-6f);

	// the module ending the segment
	seg.end();
	EXPECT_EQ(seg.classify(2, t2 + 633000, gap_max), SegmentAction::NEW_SEGMENT);
}
