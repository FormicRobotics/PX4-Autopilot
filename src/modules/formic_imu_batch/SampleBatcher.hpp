/****************************************************************************
 * Packs the samples of a sensor's FIFO messages (sensor_gyro_fifo, sensor_accel_fifo) into batches
 * of up to formic_imu_batch_s::MAX_SAMPLES samples, each with its own sample time.
 *
 * Sample times come from a sample clock. A FIFO message holds N consecutive samples, the newest at
 * about timestamp_sample, and consecutive messages hold consecutive samples: the clock counts the
 * samples on from the previous one, one sample interval each. The timestamps only steer it, through an
 * alpha-beta filter on the time of each message's newest sample (phase and sample interval): they
 * jitter by about 100 us (the ICM42688P's data-ready interrupt), more than half a sample at 8 kHz. The
 * filter starts from the driver's dt as a least-squares line through the first messages and settles to
 * fixed gains; the interval stays within 15 % of the driver's dt. A message later than the clock's prediction by more than the gap tolerance
 * (GAP_TOLERANCE_US, at least 3/4 of a sample) follows samples that never came: the clock jumps to it.
 * A message earlier than the prediction still follows the previous sample: the clock catches up. A
 * message not after the previous message is dropped.
 *
 * A sensor faster than TARGET_RATE_HZ is averaged down: each batch sample is the mean of `factor`
 * consecutive samples, stamped at their mean time, with factor = round(rate / TARGET_RATE_HZ) from
 * the driver's dt. An 8 kHz ICM42688P is averaged by 5, a 1.6 kHz BMI270 passes as it is. A group of
 * samples to average never spans a gap or a change of device: its samples are dropped. A change of
 * scale within a group (the ICM42688P switches to its coarse scale beyond 16 bit data) is averaged at
 * the coarser scale.
 *
 * A batch has one scale. Once a batch sample comes at a coarser scale, the batches stay at it until
 * SCALE_HOLD_US after the last such sample; finer samples meanwhile are converted to it. A sensor that
 * switches scale on many FIFO messages (flight near the edge of its fine range) splits a batch about
 * twice a second, not on every switch.
 *
 * A batch is emitted when it is full, and before a sample that cannot join it: another device or
 * scale, a time offset beyond uint16, or a time not after the previous sample's.
 *
 * A gap of more than 1.5 batch sample intervals between batch samples counts the batch samples
 * missing in it; each batch carries the count so far in samples_lost.
 *
 * A batch leaves the batcher with temperature NaN: the FIFO messages carry none. The module sets it
 * at publication from the driver's sensor_gyro / sensor_accel, through batchTemperature().
 *
 * No uORB: tested with gtest.
 ****************************************************************************/

#pragma once

#include <cstdint>
#include <cmath>

#include <uORB/topics/formic_imu_batch.h>

namespace formic_imu_batch
{

class SampleBatcher
{
public:
	static constexpr uint8_t MAX_SAMPLES = formic_imu_batch_s::MAX_SAMPLES;

	/// Faster sensors are averaged down to about this rate (Hz): 32 samples a batch, 50 batches a second.
	static constexpr float TARGET_RATE_HZ = 1600.f;

	/// A message later than the sample clock's prediction by more than this (at least 3/4 of a sample)
	/// follows missing samples (microseconds).
	static constexpr float GAP_TOLERANCE_US = 400.f;

	/// How long the batches stay at a coarser scale after its last sample (microseconds).
	static constexpr uint64_t SCALE_HOLD_US = 1'000'000;

	/// The number of consecutive samples averaged into one batch sample, for a sensor sampling every
	/// `dt_us` microseconds: the factor that brings it nearest TARGET_RATE_HZ, at least 1.
	static uint8_t averagingFactor(float dt_us)
	{
		if (!(dt_us > 0.f)) {
			return 1;
		}

		const long factor = lroundf(1e6f / dt_us / TARGET_RATE_HZ);
		return static_cast<uint8_t>((factor < 1) ? 1 : ((factor > MAX_FACTOR) ? MAX_FACTOR : factor));
	}

	/**
	 * Adds the samples of one FIFO message and calls emit(const formic_imu_batch_s &) for each
	 * batch that is complete. `timestamp` of an emitted batch is left for the caller.
	 * @param dt_us  time between the message's samples as the driver reports it (microseconds)
	 */
	template<typename Emit>
	void add(uint64_t timestamp_sample, float dt_us, uint32_t device_id, float scale, uint8_t samples,
		 const int16_t x[], const int16_t y[], const int16_t z[], Emit &&emit)
	{
		if (samples == 0) {
			return;
		}

		if ((device_id != _input_device_id) || (_factor == 0)) {
			// another sensor: its own rate, clock and groups
			restartInput();
			_input_device_id = device_id;
			_factor = averagingFactor(dt_us);
		}

		if ((_fits > 0) && (timestamp_sample <= _last_message_us)) {
			// a repeated or older message
			_dropped += samples;
			return;
		}

		_last_message_us = timestamp_sample;

		// the time of the message's newest sample, and the time between its samples
		uint64_t newest = timestamp_sample;
		float step = 0.f;

		if (_fits == 0) {
			_interval_us = (dt_us > 0.f) ? dt_us : 1.f;
			step = _interval_us;
			_fits = 1;

		} else {
			const float span = static_cast<float>(samples) * _interval_us;
			const float residual = static_cast<float>(static_cast<int64_t>(timestamp_sample - _clock_us)) - span;

			if (residual > gapTolerance()) {
				// samples missing: the clock jumps to the message; a group would not be consecutive samples
				step = _interval_us;
				_group_samples = 0;

			} else {
				// the samples follow the previous one; the timestamp steers the clock
				_fits++;
				const float k = static_cast<float>(_fits);
				const float alpha = fmaxf(ALPHA, 2.f * (2.f * k - 1.f) / (k * (k + 1.f)));
				const float beta = fmaxf(BETA, 6.f / (k * (k + 1.f)));

				// never standing still or running back
				const float advance = fmaxf(span + alpha * residual, 0.5f * span);
				newest = _clock_us + static_cast<uint64_t>(lroundf(advance));
				step = advance / samples;

				_interval_us += beta * residual / samples;

				if (dt_us > 0.f) {
					// within 15 % of the driver's dt (the ICM42688P's is 4.6 % short): a jittered start
					// cannot throw the clock far enough off to see a gap
					_interval_us = fminf(fmaxf(_interval_us, 0.85f * dt_us), 1.15f * dt_us);
				}
			}
		}

		_clock_us = newest;

		for (uint8_t i = 0; i < samples; i++) {
			const uint64_t back = static_cast<uint64_t>(lroundf(static_cast<float>(samples - 1 - i) * step));
			const uint64_t t = (newest > back) ? newest - back : 0;

			if (t <= _last_input_us) {
				_dropped++;
				continue;
			}

			_last_input_us = t;
			addToGroup(t, x[i], y[i], z[i], scale);

			if (_group_samples == _factor) {
				const uint64_t t_mean = _group_t0 + static_cast<uint64_t>(lroundf(static_cast<float>(_group_dt_sum) /
							_group_samples));
				addSample(t_mean, groupMean(0), groupMean(1), groupMean(2), device_id, _group_scale,
					  _interval_us * static_cast<float>(_factor), emit);
				_group_samples = 0;
			}
		}
	}

	/// Samples dropped because their time was not after the previous sample's.
	uint32_t dropped() const { return _dropped; }

	/// Batch samples emitted so far.
	uint64_t emitted() const { return _emitted; }

	/// Batch samples in the batch not yet emitted.
	uint8_t pending() const { return _batch.samples; }

	/// Samples averaged into one batch sample for the present sensor; 0 before its first message.
	uint8_t factor() const { return _factor; }

	/// The sample rate of the present sensor by its sample clock (Hz); 0 before its first message.
	float inputRateHz() const { return (_interval_us > 0.f) ? 1e6f / _interval_us : 0.f; }

	/// Emits the batch now, if it holds any sample.
	template<typename Emit>
	void flush(Emit &&emit)
	{
		if (_batch.samples > 0) {
			emitBatch(emit);
		}
	}

	/// Batch samples missing between the batch samples added, found by their times.
	uint32_t lost() const { return _lost; }

	/// Drops a partly filled batch, the samples not yet averaged and the time history (another sensor
	/// was selected); the counts stay.
	void reset()
	{
		_batch = formic_imu_batch_s{};
		_last_sample_us = 0;
		restartInput();
		_input_device_id = 0;
		_factor = 0;
	}

private:
	static constexpr uint8_t MAX_FACTOR = 32;

	// steady gains of the sample clock's alpha-beta filter: critically damped, beta = alpha^2 / (2 - alpha)
	static constexpr float ALPHA = 0.05f;
	static constexpr float BETA = ALPHA * ALPHA / (2.f - ALPHA);

	float gapTolerance() const { return fmaxf(GAP_TOLERANCE_US, 0.75f * _interval_us); }

	void restartInput()
	{
		_fits = 0;
		_last_message_us = 0;
		_clock_us = 0;
		_interval_us = 0.f;
		_last_input_us = 0;
		_group_samples = 0;
		_hold_set = false;
	}

	void addToGroup(uint64_t t, int16_t x, int16_t y, int16_t z, float scale)
	{
		const int16_t v[3] {x, y, z};
		bool rescale = false;
		float sample_to_group = 1.f;

		if (_group_samples == 0) {
			_group_t0 = t;
			_group_dt_sum = 0;
			_group_scale = scale;
			_group_sum[0] = _group_sum[1] = _group_sum[2] = 0;

		} else if (!sameScale(scale, _group_scale)) {
			if (fabsf(scale) > fabsf(_group_scale)) {
				// a coarser scale: the group so far moves to it
				for (int k = 0; k < 3; k++) {
					_group_sum[k] = lroundf(static_cast<float>(_group_sum[k]) * (_group_scale / scale));
				}

				_group_scale = scale;

			} else {
				rescale = true;
				sample_to_group = scale / _group_scale;
			}
		}

		for (int k = 0; k < 3; k++) {
			_group_sum[k] += rescale ? lroundf(static_cast<float>(v[k]) * sample_to_group) : v[k];
		}

		_group_dt_sum += t - _group_t0;
		_group_samples++;
	}

	static bool sameScale(float a, float b) { return fabsf(a - b) <= 1e-6f * fabsf(b); }

	static int16_t toInt16(float v)
	{
		const long r = lroundf(v);
		return static_cast<int16_t>((r > INT16_MAX) ? INT16_MAX : ((r < INT16_MIN) ? INT16_MIN : r));
	}

	int16_t groupMean(int axis) const
	{
		return toInt16(static_cast<float>(_group_sum[axis]) / _group_samples);
	}

	/// Adds one batch sample at time `t`; `interval_us` is the time between batch samples.
	template<typename Emit>
	void addSample(uint64_t t, int16_t x, int16_t y, int16_t z, uint32_t device_id, float scale, float interval_us,
		       Emit &emit)
	{
		if (t <= _last_sample_us) {
			_dropped++;
			return;
		}

		if ((_last_sample_us != 0) && (interval_us > 0.f)) {
			const float gap = static_cast<float>(t - _last_sample_us);

			if (gap > 1.5f * interval_us) {
				_lost += static_cast<uint32_t>(lroundf(gap / interval_us)) - 1;
			}
		}

		int16_t v[3] {x, y, z};

		if (!_hold_set || (fabsf(scale) >= fabsf(_hold_scale) * (1.f - 1e-6f)) || (t > _hold_until_us)) {
			// the first sample, a scale as coarse or coarser, or the hold ran out: this sample's scale
			_hold_scale = scale;
			_hold_until_us = t + SCALE_HOLD_US;
			_hold_set = true;

		} else {
			// a finer scale while a coarser one holds: converted to it
			for (int k = 0; k < 3; k++) {
				v[k] = toInt16(static_cast<float>(v[k]) * (scale / _hold_scale));
			}
		}

		if (_batch.samples > 0
		    && (device_id != _batch.device_id || !sameScale(_hold_scale, _batch.scale)
			|| (t - _batch.timestamp_sample) > UINT16_MAX)) {
			emitBatch(emit);
		}

		if (_batch.samples == 0) {
			_batch.timestamp_sample = t;
			_batch.device_id = device_id;
			_batch.scale = _hold_scale;
			_batch.temperature = NAN;
		}

		const uint8_t k = _batch.samples;
		_batch.dt[k] = static_cast<uint16_t>(t - _batch.timestamp_sample);
		_batch.x[k] = v[0];
		_batch.y[k] = v[1];
		_batch.z[k] = v[2];
		_batch.samples++;
		_last_sample_us = t;

		if (_batch.samples == MAX_SAMPLES) {
			emitBatch(emit);
		}
	}

	template<typename Emit>
	void emitBatch(Emit &&emit)
	{
		_batch.samples_lost = _lost;
		emit(static_cast<const formic_imu_batch_s &>(_batch));
		_emitted += _batch.samples;
		_batch = formic_imu_batch_s{};
	}

	// batches (output)
	formic_imu_batch_s _batch{};
	uint64_t _last_sample_us{0};
	uint32_t _dropped{0};
	uint32_t _lost{0};
	uint64_t _emitted{0};

	// the scale the batches hold
	bool _hold_set{false};
	float _hold_scale{0.f};
	uint64_t _hold_until_us{0};

	// FIFO samples (input) of the present sensor
	uint32_t _input_device_id{0};
	uint8_t _factor{0};

	// the sample clock: the time of the last message's newest sample and the sample interval, and the
	// number of messages the filter has taken (1: started)
	uint32_t _fits{0};
	uint64_t _last_message_us{0};
	uint64_t _clock_us{0};
	float _interval_us{0.f};
	uint64_t _last_input_us{0};

	// the samples being averaged
	uint8_t _group_samples{0};
	uint64_t _group_t0{0};
	uint64_t _group_dt_sum{0};
	float _group_scale{0.f};
	int32_t _group_sum[3] {};
};

/**
 * The temperature for a batch of `batch_device_id` (degrees Celsius), from a driver report
 * (sensor_gyro or sensor_accel) of `report_device_id`: NaN when the report is of another device,
 * there is no device (0), or the driver has no valid reading (NaN).
 */
inline float batchTemperature(uint32_t batch_device_id, uint32_t report_device_id, float report_temperature)
{
	if ((batch_device_id == 0) || (report_device_id != batch_device_id) || !std::isfinite(report_temperature)) {
		return NAN;
	}

	return report_temperature;
}

} // namespace formic_imu_batch
