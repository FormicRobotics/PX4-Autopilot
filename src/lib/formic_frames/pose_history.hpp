/****************************************************************************
 * A short history of attitudes and positions, read back at any time inside it.
 ****************************************************************************/

#pragma once

#include <cstdint>

#include <matrix/math.hpp>

namespace formic_frames
{

struct PoseSample {
	uint64_t time_us{0};
	matrix::Quatf q{};
	matrix::Vector3f p{};
	matrix::Vector3f v{};
};

/**
 * The last N samples, in time order. Older samples are dropped.
 */
template<int N>
class PoseHistory
{
public:
	void clear() { _count = 0; }

	bool empty() const { return _count == 0; }

	const PoseSample &newest() const { return at(_count - 1); }

	/// Adds a sample. One that is not newer than the newest is dropped.
	void push(const PoseSample &s)
	{
		if (_count > 0 && s.time_us <= newest().time_us) {
			return;
		}

		_head = (_head + 1) % N;
		_buf[_head] = s;

		if (_count < N) {
			_count++;
		}
	}

	/**
	 * Returns the sample at `time_us`: interpolated between the two around it, the oldest when
	 * older, and the newest moved on by its velocity when newer (by at most `max_extrapolation_us`).
	 * @return false when empty or newer than the newest by more than max_extrapolation_us
	 */
	bool sampleAt(uint64_t time_us, PoseSample &out, uint64_t max_extrapolation_us = 50000) const
	{
		if (_count == 0) {
			return false;
		}

		const PoseSample &last = newest();

		if (time_us >= last.time_us) {
			if (time_us - last.time_us > max_extrapolation_us) {
				return false;
			}

			out = last;
			out.p = last.p + last.v * (static_cast<float>(time_us - last.time_us) * 1e-6f);
			out.time_us = time_us;
			return true;
		}

		for (int i = _count - 1; i > 0; i--) {
			const PoseSample &b = at(i);
			const PoseSample &a = at(i - 1);

			if (a.time_us <= time_us) {
				const float f = static_cast<float>(time_us - a.time_us) / static_cast<float>(b.time_us - a.time_us);
				out = interpolate(a, b, f);
				out.time_us = time_us;
				return true;
			}
		}

		out = at(0);
		return true;
	}

	static PoseSample interpolate(const PoseSample &a, const PoseSample &b, float f)
	{
		PoseSample s;
		s.time_us = a.time_us + static_cast<uint64_t>(f * static_cast<float>(b.time_us - a.time_us));
		// the shorter way round, then a normalised linear blend: the samples are close
		const float sign = (a.q.dot(b.q) < 0.f) ? -1.f : 1.f;
		s.q = matrix::Quatf(a.q(0) * (1.f - f) + sign * b.q(0) * f,
				    a.q(1) * (1.f - f) + sign * b.q(1) * f,
				    a.q(2) * (1.f - f) + sign * b.q(2) * f,
				    a.q(3) * (1.f - f) + sign * b.q(3) * f);
		s.q.normalize();
		s.p = a.p * (1.f - f) + b.p * f;
		s.v = a.v * (1.f - f) + b.v * f;
		return s;
	}

private:
	/// i = 0 is the oldest
	const PoseSample &at(int i) const { return _buf[(_head + N - (_count - 1) + i) % N]; }

	PoseSample _buf[N] {};
	int _head{N - 1};
	int _count{0};
};

} // namespace formic_frames
