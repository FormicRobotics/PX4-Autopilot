/****************************************************************************
 * Packs the samples of a sensor's FIFO messages (sensor_gyro_fifo, sensor_accel_fifo) into batches
 * of up to formic_imu_batch_s::MAX_SAMPLES samples, each with its own sample time.
 *
 * A FIFO message holds N samples spaced dt apart, the newest at timestamp_sample:
 *
 *     t_i = timestamp_sample - (N - 1 - i) * dt
 *
 * A batch is emitted when it is full, and before a sample that cannot join it: another device or
 * scale, a time offset beyond uint16, or a time not after the previous sample's.
 *
 * A gap of more than 1.5 dt to the previous sample counts the samples missing in it; each batch
 * carries the count so far in samples_lost.
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

	/**
	 * Adds the samples of one FIFO message and calls emit(const formic_imu_batch_s &) for each
	 * batch that is complete. `timestamp` of an emitted batch is left for the caller.
	 * @param dt_us  time between the message's samples (microseconds)
	 */
	template<typename Emit>
	void add(uint64_t timestamp_sample, float dt_us, uint32_t device_id, float scale, uint8_t samples,
		 const int16_t x[], const int16_t y[], const int16_t z[], Emit &&emit)
	{
		for (uint8_t i = 0; i < samples; i++) {
			const uint64_t back = static_cast<uint64_t>(lroundf(static_cast<float>(samples - 1 - i) * dt_us));
			const uint64_t t = (timestamp_sample > back) ? timestamp_sample - back : 0;

			if (t <= _last_sample_us) {
				// a repeated or older sample
				_dropped++;
				continue;
			}

			if ((_last_sample_us != 0) && (dt_us > 0.f)) {
				const float gap = static_cast<float>(t - _last_sample_us);

				if (gap > 1.5f * dt_us) {
					_lost += static_cast<uint32_t>(lroundf(gap / dt_us)) - 1;
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
			}

			const uint8_t k = _batch.samples;
			_batch.dt[k] = static_cast<uint16_t>(t - _batch.timestamp_sample);
			_batch.x[k] = x[i];
			_batch.y[k] = y[i];
			_batch.z[k] = z[i];
			_batch.samples++;
			_last_sample_us = t;

			if (_batch.samples == MAX_SAMPLES) {
				emitBatch(emit);
			}
		}
	}

	/// Samples dropped because their time was not after the previous sample's.
	uint32_t dropped() const { return _dropped; }

	/// Samples emitted so far.
	uint64_t emitted() const { return _emitted; }

	/// Samples in the batch not yet emitted.
	uint8_t pending() const { return _batch.samples; }

	/// Emits the batch now, if it holds any sample.
	template<typename Emit>
	void flush(Emit &&emit)
	{
		if (_batch.samples > 0) {
			emitBatch(emit);
		}
	}

	/// Samples missing between the samples added, found by their times.
	uint32_t lost() const { return _lost; }

	/// Drops a partly filled batch and the time history (another sensor was selected); the counts stay.
	void reset()
	{
		_batch = formic_imu_batch_s{};
		_last_sample_us = 0;
	}

private:
	template<typename Emit>
	void emitBatch(Emit &&emit)
	{
		_batch.samples_lost = _lost;
		emit(static_cast<const formic_imu_batch_s &>(_batch));
		_emitted += _batch.samples;
		_batch = formic_imu_batch_s{};
	}

	formic_imu_batch_s _batch{};
	uint64_t _last_sample_us{0};
	uint32_t _dropped{0};
	uint32_t _lost{0};
	uint64_t _emitted{0};
};

} // namespace formic_imu_batch
