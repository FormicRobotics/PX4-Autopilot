/****************************************************************************
 * Packs the samples of a sensor's FIFO messages (sensor_gyro_fifo, sensor_accel_fifo) into batches
 * of up to formic_imu_batch_s::MAX_SAMPLES samples, each with its own sample time.
 *
 * A FIFO message holds N samples one sample interval apart, the newest at timestamp_sample:
 *
 *     t_i = timestamp_sample - (N - 1 - i) * interval
 *
 * The interval is measured: the time between the newest samples of two consecutive messages over the
 * samples of the later one, smoothed. The driver's dt is only the first guess and the bound for a
 * measurement (from 0.8 to 1.25 dt; a lost message gives a longer time and is not taken): an
 * ICM42688P reports 119.5 us from its FIFO clock while its samples come every 124.4 us.
 *
 * A sensor faster than TARGET_RATE_HZ is averaged down: each batch sample is the mean of `factor`
 * consecutive samples, stamped at their mean time, with factor = round(rate / TARGET_RATE_HZ) from
 * the driver's dt. An 8 kHz ICM42688P is averaged by 5, a 1.6 kHz BMI270 passes as it is. A group of
 * samples to average never spans a gap or a change of device: its samples are dropped. A change of
 * scale within a group (the ICM42688P accel switches between 16 and 20 bit data) is averaged at the
 * coarser scale.
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
			// another sensor: its own rate, interval and groups
			restartInput();
			_input_device_id = device_id;
			_factor = averagingFactor(dt_us);
		}

		const float interval = updateInterval(timestamp_sample, dt_us, samples);

		for (uint8_t i = 0; i < samples; i++) {
			const uint64_t back = static_cast<uint64_t>(lroundf(static_cast<float>(samples - 1 - i) * interval));
			const uint64_t t = (timestamp_sample > back) ? timestamp_sample - back : 0;

			if (t <= _last_input_us) {
				// a repeated or older sample
				_dropped++;
				continue;
			}

			if ((_group_samples > 0) && (static_cast<float>(t - _last_input_us) > 1.5f * interval)) {
				// samples missing: the group would not be consecutive samples
				_group_samples = 0;
			}

			_last_input_us = t;
			addToGroup(t, x[i], y[i], z[i], scale);

			if (_group_samples == _factor) {
				const uint64_t t_mean = _group_t0 + static_cast<uint64_t>(lroundf(static_cast<float>(_group_dt_sum) /
							_group_samples));
				addSample(t_mean, groupMean(0), groupMean(1), groupMean(2), device_id, _group_scale,
					  interval * static_cast<float>(_factor), emit);
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

	/// The measured sample rate of the present sensor (Hz); 0 before it is measured.
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

	/// Weight of a new interval measurement in the smoothed interval.
	static constexpr float INTERVAL_GAIN = 0.01f;

	void restartInput()
	{
		_last_message_us = 0;
		_interval_us = 0.f;
		_last_input_us = 0;
		_group_samples = 0;
	}

	/// Measures the sample interval with the message's timestamp and returns the interval to use.
	float updateInterval(uint64_t timestamp_sample, float dt_us, uint8_t samples)
	{
		if ((_last_message_us != 0) && (timestamp_sample > _last_message_us) && (dt_us > 0.f)) {
			const float measured = static_cast<float>(timestamp_sample - _last_message_us) / samples;

			if ((measured > 0.8f * dt_us) && (measured < 1.25f * dt_us)) {
				_interval_us = (_interval_us > 0.f) ? _interval_us + INTERVAL_GAIN * (measured - _interval_us) : measured;
			}
		}

		if (timestamp_sample > _last_message_us) {
			_last_message_us = timestamp_sample;
		}

		return (_interval_us > 0.f) ? _interval_us : dt_us;
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

		} else if (fabsf(scale - _group_scale) > 1e-6f * fabsf(_group_scale)) {
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

	int16_t groupMean(int axis) const
	{
		const long mean = lroundf(static_cast<float>(_group_sum[axis]) / _group_samples);
		return static_cast<int16_t>((mean > INT16_MAX) ? INT16_MAX : ((mean < INT16_MIN) ? INT16_MIN : mean));
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

		if (_batch.samples > 0
		    && (device_id != _batch.device_id || fabsf(scale - _batch.scale) > 1e-6f * fabsf(_batch.scale)
			|| (t - _batch.timestamp_sample) > UINT16_MAX)) {
			emitBatch(emit);
		}

		if (_batch.samples == 0) {
			_batch.timestamp_sample = t;
			_batch.device_id = device_id;
			_batch.scale = scale;
			_batch.temperature = NAN;
		}

		const uint8_t k = _batch.samples;
		_batch.dt[k] = static_cast<uint16_t>(t - _batch.timestamp_sample);
		_batch.x[k] = x;
		_batch.y[k] = y;
		_batch.z[k] = z;
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

	// FIFO samples (input) of the present sensor
	uint32_t _input_device_id{0};
	uint8_t _factor{0};
	uint64_t _last_message_us{0};
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
