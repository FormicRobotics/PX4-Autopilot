/****************************************************************************
 * The VIO pose carried to now on the IMU.
 *
 * Modelled on EKF2's OutputPredictor, but indexed by time: a VIO pose arrives 70-125 ms after
 * the time it is at, and that delay changes from pose to pose. Every IMU sample is integrated
 * into the output and kept, with the output after it, for the length of the history. A VIO
 * pose is compared with the output at its own time and the difference is pulled out of the
 * output with the time constants tau_att, tau_vel and tau_pos.
 *
 * Every correction is also applied to the stored outputs, so the next pose is compared with an
 * output that already contains it and the same error is not removed twice.
 *
 * The state is of the IMU point in a gravity-aligned NED frame. No uORB: tested with gtest.
 ****************************************************************************/

#pragma once

#include <cstdint>

#include <matrix/math.hpp>

namespace formic_vio_est
{

/// One IMU sample, bias corrected, in the body frame.
struct ImuDelta {
	uint64_t time_us{0};          ///< end of the integration period
	matrix::Vector3f delta_ang{}; ///< rad
	matrix::Vector3f delta_vel{}; ///< m/s
	float dt{0.f};                ///< s
};

struct OutputState {
	uint64_t time_us{0};
	matrix::Quatf q{};       ///< body to earth
	matrix::Vector3f v{};    ///< m/s
	matrix::Vector3f p{};    ///< m
};

/// The output minus a VIO pose, at the time of the pose.
struct TrackingError {
	float att{0.f};  ///< rad
	float vel{0.f};  ///< m/s
	float pos{0.f};  ///< m
};

enum class PropagatorResult : uint8_t {
	ACCEPTED,
	TOO_OLD,     ///< older than the IMU history
	IN_FUTURE,   ///< newer than the newest IMU sample
	NOT_INITIALISED,
};

template<int N>
class OutputPropagator
{
public:
	static constexpr float GRAVITY{9.80665f};
	/// A pose newer than the newest IMU sample by up to this much is taken at the newest sample.
	static constexpr uint64_t FUTURE_TOLERANCE_US{10000};

	void setTimeConstants(float tau_att, float tau_vel, float tau_pos)
	{
		_tau_att = tau_att;
		_tau_vel = tau_vel;
		_tau_pos = tau_pos;
	}

	bool initialised() const { return _initialised; }

	/// Forgets the output. The IMU history is kept.
	void reset()
	{
		_initialised = false;
		_corr_att.zero();
		_corr_vel.zero();
		_corr_pos.zero();
	}

	const OutputState &output() const { return _entries[_head].state; }

	/// Acceleration of the IMU point in the earth frame over the newest IMU sample, m/s^2.
	const matrix::Vector3f &acceleration() const { return _accel; }

	/// Body rate over the newest IMU sample, rad/s.
	const matrix::Vector3f &angularRate() const { return _rate; }

	uint64_t newestImuTime() const { return _count > 0 ? _entries[_head].imu.time_us : 0; }

	/// The earliest time the history can be read at.
	uint64_t oldestTime() const { return _count > 0 ? oldest().imu.time_us : 0; }

	/**
	 * Integrates one IMU sample and stores it. Before the output has been started, only stores it.
	 */
	void update(const ImuDelta &imu)
	{
		if (_count > 0 && imu.time_us <= _entries[_head].imu.time_us) {
			return; // not newer: a repeated or out-of-order sample
		}

		const OutputState prev = output();
		const bool valid = _initialised && _count > 0;

		_head = (_head + 1) % N;

		if (_count < N) {
			_count++;
		}

		Entry &e = _entries[_head];
		e.imu = imu;
		e.valid = valid;

		if (!valid) {
			e.state = OutputState{};
			e.state.time_us = imu.time_us;

			if (_valid_count > 0) {
				_valid_count = 0;
			}

			return;
		}

		e.state = integrate(prev, imu, 1.f);

		if (_valid_count < _count) {
			_valid_count++;
		}

		applyCorrection(imu.dt);
	}

	/**
	 * Starts the output from a pose at `time_us`: the output at that time is the pose, then the
	 * stored IMU samples after it are integrated to now.
	 */
	PropagatorResult initialise(uint64_t time_us, const matrix::Quatf &q, const matrix::Vector3f &v,
				    const matrix::Vector3f &p)
	{
		int j = 0;
		const PropagatorResult r = findAfter(time_us, j, false);

		if (r != PropagatorResult::ACCEPTED) {
			return r;
		}

		OutputState s;
		s.time_us = time_us;
		s.q = q.normalized();
		s.v = v;
		s.p = p;

		// part of sample j after time_us, then every sample after it
		for (int i = j; i < _count; i++) {
			Entry &e = at(i);
			float fraction = 1.f;

			if (i == j && e.imu.dt > 0.f && e.imu.time_us > time_us) {
				fraction = math_constrain(static_cast<float>(e.imu.time_us - time_us) * 1e-6f / e.imu.dt, 0.f, 1.f);

			} else if (i == j) {
				fraction = 0.f;
			}

			s = integrate(s, e.imu, fraction);
			e.state = s;
			e.valid = true;
		}

		for (int i = 0; i < j; i++) {
			at(i).valid = false;
		}

		_valid_count = _count - j;
		_initialised = true;
		_corr_att.zero();
		_corr_vel.zero();
		_corr_pos.zero();
		return PropagatorResult::ACCEPTED;
	}

	/**
	 * Returns the output at `time_us`, interpolated between the stored outputs around it.
	 */
	PropagatorResult stateAt(uint64_t time_us, OutputState &out) const
	{
		if (!_initialised) {
			return PropagatorResult::NOT_INITIALISED;
		}

		int j = 0;
		const PropagatorResult r = findAfter(time_us, j, true);

		if (r != PropagatorResult::ACCEPTED) {
			return r;
		}

		const Entry &b = at(j);

		if (b.imu.time_us <= time_us || j == 0 || !at(j - 1).valid) {
			out = b.state;

			if (b.imu.time_us > time_us && (j == 0 || !at(j - 1).valid)) {
				return PropagatorResult::TOO_OLD;
			}

			return PropagatorResult::ACCEPTED;
		}

		const Entry &a = at(j - 1);
		const float f = static_cast<float>(time_us - a.imu.time_us) / static_cast<float>(b.imu.time_us - a.imu.time_us);
		out = blend(a.state, b.state, f);
		out.time_us = time_us;
		return PropagatorResult::ACCEPTED;
	}

	/**
	 * Pulls the output toward a pose at `time_us`. The error at that time replaces the previous
	 * one and is removed over the next IMU samples.
	 * @param vel_valid false: the velocity is not corrected
	 */
	PropagatorResult correct(uint64_t time_us, const matrix::Quatf &q, const matrix::Vector3f &v, bool vel_valid,
				 const matrix::Vector3f &p, TrackingError *error = nullptr)
	{
		OutputState s;
		const PropagatorResult r = stateAt(time_us, s);

		if (r != PropagatorResult::ACCEPTED) {
			return r;
		}

		matrix::Quatf dq = (matrix::Quatf(q.normalized()) * s.q.inversed()).normalized();

		if (dq(0) < 0.f) {
			dq = matrix::Quatf(-dq(0), -dq(1), -dq(2), -dq(3));
		}

		_corr_att = matrix::AxisAnglef(dq);
		_corr_vel = vel_valid ? (v - s.v) : matrix::Vector3f{};
		_corr_pos = p - s.p;

		if (error) {
			error->att = _corr_att.norm();
			error->vel = vel_valid ? _corr_vel.norm() : 0.f;
			error->pos = _corr_pos.norm();
		}

		return PropagatorResult::ACCEPTED;
	}

private:
	struct Entry {
		ImuDelta imu{};
		OutputState state{}; ///< output at imu.time_us, after integrating imu
		bool valid{false};
	};

	static float math_constrain(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

	Entry &at(int i) { return _entries[(_head + N - (_count - 1) + i) % N]; }
	const Entry &at(int i) const { return _entries[(_head + N - (_count - 1) + i) % N]; }
	const Entry &oldest() const { return at(0); }

	/**
	 * Finds the first stored sample whose end is at or after `time_us`.
	 * @param need_valid the outputs must be valid back to that sample
	 */
	PropagatorResult findAfter(uint64_t time_us, int &index, bool need_valid) const
	{
		if (_count == 0) {
			return PropagatorResult::TOO_OLD;
		}

		const Entry &newest = at(_count - 1);

		if (time_us > newest.imu.time_us) {
			if (time_us - newest.imu.time_us > FUTURE_TOLERANCE_US) {
				return PropagatorResult::IN_FUTURE;
			}

			index = _count - 1;
			return PropagatorResult::ACCEPTED;
		}

		const int first = need_valid ? (_count - _valid_count) : 0;

		// the start of the oldest usable sample
		const Entry &first_entry = at(first);
		const uint64_t first_dt_us = static_cast<uint64_t>(first_entry.imu.dt * 1e6f);
		const uint64_t earliest = need_valid ? first_entry.imu.time_us
					  : (first_entry.imu.time_us > first_dt_us ? first_entry.imu.time_us - first_dt_us : 0);

		if (time_us < earliest) {
			return PropagatorResult::TOO_OLD;
		}

		for (int i = _count - 1; i >= first; i--) {
			if (at(i).imu.time_us < time_us) {
				index = i + 1;
				return PropagatorResult::ACCEPTED;
			}
		}

		index = first;
		return PropagatorResult::ACCEPTED;
	}

	/// Integrates `fraction` of one IMU sample.
	OutputState integrate(const OutputState &s, const ImuDelta &imu, float fraction)
	{
		OutputState n = s;
		n.time_us = imu.time_us;

		if (!(fraction > 0.f) || !(imu.dt > 0.f)) {
			return n;
		}

		const float dt = imu.dt * fraction;
		const matrix::Vector3f dang = imu.delta_ang * fraction;
		const matrix::Vector3f dvel = imu.delta_vel * fraction;

		// rotate the velocity change with the attitude half way through the sample
		const matrix::Quatf q_mid = (s.q * matrix::Quatf(matrix::AxisAnglef(dang * 0.5f))).normalized();
		n.q = (s.q * matrix::Quatf(matrix::AxisAnglef(dang))).normalized();

		const matrix::Vector3f dvel_earth = matrix::Dcmf(q_mid) * dvel + matrix::Vector3f(0.f, 0.f, GRAVITY * dt);
		n.v = s.v + dvel_earth;
		n.p = s.p + (s.v + n.v) * (0.5f * dt);

		_accel = dvel_earth / dt;
		_rate = dang / dt;
		return n;
	}

	static OutputState blend(const OutputState &a, const OutputState &b, float f)
	{
		OutputState s;
		const float sign = (a.q.dot(b.q) < 0.f) ? -1.f : 1.f;
		s.q = matrix::Quatf(a.q(0) * (1.f - f) + sign * b.q(0) * f,
				    a.q(1) * (1.f - f) + sign * b.q(1) * f,
				    a.q(2) * (1.f - f) + sign * b.q(2) * f,
				    a.q(3) * (1.f - f) + sign * b.q(3) * f);
		s.q.normalize();
		s.v = a.v * (1.f - f) + b.v * f;
		s.p = a.p * (1.f - f) + b.p * f;
		return s;
	}

	/// Removes part of the remaining error over `dt`, from the output and from every stored output.
	void applyCorrection(float dt)
	{
		const float k_att = gain(dt, _tau_att);
		const float k_vel = gain(dt, _tau_vel);
		const float k_pos = gain(dt, _tau_pos);

		const matrix::Vector3f d_att = _corr_att * k_att;
		const matrix::Vector3f d_vel = _corr_vel * k_vel;
		const matrix::Vector3f d_pos = _corr_pos * k_pos;

		_corr_att -= d_att;
		_corr_vel -= d_vel;
		_corr_pos -= d_pos;

		const bool rotate = d_att.norm_squared() > 0.f;
		const matrix::Quatf dq = rotate ? matrix::Quatf(matrix::AxisAnglef(d_att)) : matrix::Quatf{};

		for (int i = _count - _valid_count; i < _count; i++) {
			OutputState &s = at(i).state;

			if (rotate) {
				s.q = (dq * s.q).normalized();
			}

			s.v += d_vel;
			s.p += d_pos;
		}
	}

	/// Fraction of the remaining error removed over `dt`: 1 - exp(-dt / tau). Never overshoots.
	static float gain(float dt, float tau)
	{
		if (!(tau > 0.f)) {
			return 1.f;
		}

		return 1.f - expf(-dt / tau);
	}

	Entry _entries[N] {};
	int _head{N - 1};
	int _count{0};
	int _valid_count{0}; ///< the newest _valid_count entries have a valid output

	bool _initialised{false};

	float _tau_att{0.1f};
	float _tau_vel{0.25f};
	float _tau_pos{0.25f};

	matrix::Vector3f _corr_att{}; ///< remaining error, earth-frame rotation vector, rad
	matrix::Vector3f _corr_vel{}; ///< remaining error, m/s
	matrix::Vector3f _corr_pos{}; ///< remaining error, m

	matrix::Vector3f _accel{};
	matrix::Vector3f _rate{};
};

} // namespace formic_vio_est
