#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "BatchGate.hpp"
#include "SampleBatcher.hpp"

using formic_imu_batch::BatchGate;
using formic_imu_batch::SampleBatcher;

namespace
{
constexpr uint32_t kDevice = 3604490;     // BMI270 on SPI1, as on the bench drone
constexpr float kScale = 0.00106f;
constexpr float kDtUs = 625.f;           // 1.6 kHz
constexpr uint64_t kStartUs = 10'000'000;

struct Sink {
	std::vector<formic_imu_batch_s> batches;
	void operator()(const formic_imu_batch_s &b) { batches.push_back(b); }
};

// one FIFO message of `n` samples, the newest at `t_us`; sample i has x = first + i
void addMessage(SampleBatcher &batcher, Sink &sink, uint64_t t_us, uint8_t n, int16_t first,
		uint32_t device = kDevice, float scale = kScale)
{
	int16_t x[32] {}, y[32] {}, z[32] {};

	for (int i = 0; i < n; i++) {
		x[i] = static_cast<int16_t>(first + i);
		y[i] = static_cast<int16_t>(-(first + i));
		z[i] = 7;
	}

	batcher.add(t_us, kDtUs, device, scale, n, x, y, z, sink);
}
}

// One sample per FIFO message, as the BMI270 publishes: 32 messages make one full batch, with the
// first sample's time and one offset per sample.
TEST(FormicImuBatch, OneSamplePerMessageFillsABatch)
{
	SampleBatcher batcher;
	Sink sink;

	for (int i = 0; i < 32; i++) {
		addMessage(batcher, sink, kStartUs + static_cast<uint64_t>(i * kDtUs), 1, static_cast<int16_t>(i));
	}

	ASSERT_EQ(sink.batches.size(), 1u);
	const formic_imu_batch_s &b = sink.batches[0];
	EXPECT_EQ(b.samples, 32);
	EXPECT_EQ(b.timestamp_sample, kStartUs);
	EXPECT_EQ(b.device_id, kDevice);
	EXPECT_EQ(b.scale, kScale);
	EXPECT_EQ(b.dt[0], 0);
	EXPECT_EQ(b.dt[31], 31 * 625);
	EXPECT_EQ(b.x[31], 31);
	EXPECT_EQ(b.y[31], -31);
	EXPECT_EQ(b.z[0], 7);
	EXPECT_EQ(batcher.emitted(), 32u);
}

// A message of 8 samples: the newest is at timestamp_sample, the others dt before it.
TEST(FormicImuBatch, SamplesOfOneMessageAreSpacedBack)
{
	SampleBatcher batcher;
	Sink sink;

	for (int m = 0; m < 4; m++) {
		addMessage(batcher, sink, kStartUs + static_cast<uint64_t>((m * 8 + 7) * kDtUs), 8, static_cast<int16_t>(m * 8));
	}

	ASSERT_EQ(sink.batches.size(), 1u);
	const formic_imu_batch_s &b = sink.batches[0];
	EXPECT_EQ(b.timestamp_sample, kStartUs);

	for (int i = 0; i < 32; i++) {
		EXPECT_EQ(b.dt[i], i * 625);
		EXPECT_EQ(b.x[i], i);
	}
}

// A lost FIFO message leaves a gap of two intervals in the offsets; nothing else changes.
TEST(FormicImuBatch, ALostMessageShowsAsAGap)
{
	SampleBatcher batcher;
	Sink sink;

	for (int i = 0; i < 33; i++) {
		if (i == 10) {
			continue;
		}

		addMessage(batcher, sink, kStartUs + static_cast<uint64_t>(i * kDtUs), 1, static_cast<int16_t>(i));
	}

	ASSERT_EQ(sink.batches.size(), 1u);
	EXPECT_EQ(sink.batches[0].dt[9], 9 * 625);
	EXPECT_EQ(sink.batches[0].dt[10], 11 * 625);
	EXPECT_EQ(batcher.lost(), 1u);
	EXPECT_EQ(sink.batches[0].samples_lost, 1u);
}

// The BMI270 delivers every 630 us while the driver reports the nominal 625 us: a steady stream counts
// nothing as missing, a gap of two or four intervals counts one or three; the count is cumulative.
TEST(FormicImuBatch, MissingSamplesAreCountedByTime)
{
	SampleBatcher batcher;
	Sink sink;
	constexpr uint64_t kRealDtUs = 630;
	uint64_t t = kStartUs;

	for (int i = 0; i < 32; i++) {
		addMessage(batcher, sink, t, 1, static_cast<int16_t>(i));
		t += kRealDtUs;
	}

	ASSERT_EQ(sink.batches.size(), 1u);
	EXPECT_EQ(sink.batches[0].samples_lost, 0u);

	t += kRealDtUs;			// one missing
	addMessage(batcher, sink, t, 1, 100);
	t += 4 * kRealDtUs;		// three missing

	for (int i = 0; i < 31; i++) {
		addMessage(batcher, sink, t, 1, static_cast<int16_t>(101 + i));
		t += kRealDtUs;
	}

	ASSERT_EQ(sink.batches.size(), 2u);
	EXPECT_EQ(sink.batches[1].samples_lost, 4u);
	EXPECT_EQ(batcher.lost(), 4u);
}

// A new sensor starts without a time history: the jump to its first sample is not counted.
TEST(FormicImuBatch, ResetForgetsTheLastSampleTime)
{
	SampleBatcher batcher;
	Sink sink;
	addMessage(batcher, sink, kStartUs, 1, 1);
	batcher.reset();
	addMessage(batcher, sink, kStartUs + 10'000, 1, 2);
	EXPECT_EQ(batcher.lost(), 0u);
}

// A repeated or older sample is dropped and counted.
TEST(FormicImuBatch, OldSamplesAreDropped)
{
	SampleBatcher batcher;
	Sink sink;
	addMessage(batcher, sink, kStartUs, 1, 1);
	addMessage(batcher, sink, kStartUs, 1, 2);
	addMessage(batcher, sink, kStartUs - 625, 1, 3);
	EXPECT_EQ(batcher.dropped(), 2u);
}

// Another device or scale closes the batch first, so a batch never mixes sensors or scales.
TEST(FormicImuBatch, DeviceOrScaleChangeClosesTheBatch)
{
	SampleBatcher batcher;
	Sink sink;
	addMessage(batcher, sink, kStartUs, 1, 1);
	addMessage(batcher, sink, kStartUs + 625, 1, 2);
	addMessage(batcher, sink, kStartUs + 1250, 1, 3, kDevice, 2.f * kScale);
	ASSERT_EQ(sink.batches.size(), 1u);
	EXPECT_EQ(sink.batches[0].samples, 2);

	addMessage(batcher, sink, kStartUs + 1875, 1, 4, 2490386, 2.f * kScale);
	ASSERT_EQ(sink.batches.size(), 2u);
	EXPECT_EQ(sink.batches[1].samples, 1);
	EXPECT_EQ(sink.batches[1].scale, 2.f * kScale);
}

// A gap longer than a uint16 offset can hold closes the batch.
TEST(FormicImuBatch, LongGapClosesTheBatch)
{
	SampleBatcher batcher;
	Sink sink;
	addMessage(batcher, sink, kStartUs, 1, 1);
	addMessage(batcher, sink, kStartUs + 70'000, 1, 2);
	ASSERT_EQ(sink.batches.size(), 1u);
	EXPECT_EQ(sink.batches[0].samples, 1);
}

// flush() emits a part-filled batch (the module closes the accel batch early to stagger it from the
// gyro batch); the next sample starts a new batch.
TEST(FormicImuBatch, FlushEmitsAPartBatch)
{
	SampleBatcher batcher;
	Sink sink;

	for (int i = 0; i < 16; i++) {
		addMessage(batcher, sink, kStartUs + static_cast<uint64_t>(i * kDtUs), 1, static_cast<int16_t>(i));
	}

	EXPECT_EQ(batcher.pending(), 16);
	batcher.flush(sink);
	ASSERT_EQ(sink.batches.size(), 1u);
	EXPECT_EQ(sink.batches[0].samples, 16);
	EXPECT_EQ(batcher.pending(), 0);

	batcher.flush(sink);
	EXPECT_EQ(sink.batches.size(), 1u);

	addMessage(batcher, sink, kStartUs + static_cast<uint64_t>(16 * kDtUs), 1, 16);
	batcher.flush(sink);
	ASSERT_EQ(sink.batches.size(), 2u);
	EXPECT_EQ(sink.batches[1].timestamp_sample, kStartUs + static_cast<uint64_t>(16 * kDtUs));
	EXPECT_EQ(batcher.emitted(), 17u);
	EXPECT_EQ(batcher.lost(), 0u);
}

// A batch leaves the batcher with temperature NaN, never 0 degC: the FIFO messages carry none, and the
// module sets it at publication.
TEST(FormicImuBatch, BatchTemperatureIsUnknownUntilSet)
{
	SampleBatcher batcher;
	Sink sink;

	for (int i = 0; i < 40; i++) {
		addMessage(batcher, sink, kStartUs + static_cast<uint64_t>(i * kDtUs), 1, static_cast<int16_t>(i));
	}

	batcher.flush(sink);
	ASSERT_EQ(sink.batches.size(), 2u);
	EXPECT_TRUE(std::isnan(sink.batches[0].temperature));
	EXPECT_TRUE(std::isnan(sink.batches[1].temperature));
}

// A batch takes the temperature of a report (sensor_gyro / sensor_accel) of its own device; another
// device, no device or no valid reading give NaN.
TEST(FormicImuBatch, TemperatureComesFromTheBatchDevice)
{
	using formic_imu_batch::batchTemperature;
	constexpr uint32_t kOtherDevice = 2490386;

	EXPECT_EQ(batchTemperature(kDevice, kDevice, 41.5f), 41.5f);
	EXPECT_EQ(batchTemperature(kDevice, kDevice, -5.25f), -5.25f);
	EXPECT_TRUE(std::isnan(batchTemperature(kDevice, kOtherDevice, 41.5f)));
	EXPECT_TRUE(std::isnan(batchTemperature(0, 0, 41.5f)));
	EXPECT_TRUE(std::isnan(batchTemperature(kDevice, kDevice, NAN)));	// BMI270 register invalid
	EXPECT_TRUE(std::isnan(batchTemperature(kDevice, kDevice, INFINITY)));
}

// Turned off and on again (the module resets the batcher when the gate turns off): the first batch
// holds only samples from after the pause, and the pause is not counted as lost samples.
TEST(FormicImuBatch, OffAndOnAgainStartsClean)
{
	SampleBatcher batcher;
	Sink sink;

	for (int i = 0; i < 10; i++) {
		addMessage(batcher, sink, kStartUs + static_cast<uint64_t>(i * kDtUs), 1, static_cast<int16_t>(i));
	}

	batcher.reset();
	EXPECT_EQ(batcher.pending(), 0);

	constexpr uint64_t kResumeUs = kStartUs + 5'000'000;

	for (int i = 0; i < 32; i++) {
		addMessage(batcher, sink, kResumeUs + static_cast<uint64_t>(i * kDtUs), 1, static_cast<int16_t>(100 + i));
	}

	ASSERT_EQ(sink.batches.size(), 1u);
	EXPECT_EQ(sink.batches[0].samples, 32);
	EXPECT_EQ(sink.batches[0].timestamp_sample, kResumeUs);
	EXPECT_EQ(sink.batches[0].x[0], 100);
	EXPECT_EQ(sink.batches[0].samples_lost, 0u);
	EXPECT_EQ(batcher.lost(), 0u);
}

namespace
{
constexpr uint32_t kIcmDevice = 2490386;  // ICM42688P on SPI2, as on the bench drone
constexpr float kIcmScale = 0.00013f;
constexpr uint64_t kIcmIntervalUs = 125;  // 8 kHz

// FIFO message `m` of an 8 kHz ICM42688P read every 1 ms: input samples n = 8m ... 8m + 7 at
// kStartUs + n * 125 us, the newest at timestamp_sample; sample n has x = n, y = -n, z = `z`.
void addIcmMessage(SampleBatcher &batcher, Sink &sink, int m, float reported_dt_us, float scale = kIcmScale,
		   int16_t x_value = INT16_MIN, int16_t z = 1000)
{
	int16_t x[32] {}, y[32] {}, zs[32] {};

	for (int i = 0; i < 8; i++) {
		const int n = 8 * m + i;
		x[i] = (x_value == INT16_MIN) ? static_cast<int16_t>(n) : x_value;
		y[i] = static_cast<int16_t>(-x[i]);
		zs[i] = z;
	}

	batcher.add(kStartUs + static_cast<uint64_t>(8 * m + 7) * kIcmIntervalUs, reported_dt_us, kIcmDevice, scale, 8, x, y,
		    zs, sink);
}

// Every batch sample is the mean of consecutive input samples when its value and its time both point
// at the same input sample: x == (time - kStartUs) / 125 us.
void expectMeansOfConsecutiveSamples(const formic_imu_batch_s &b)
{
	for (int k = 0; k < b.samples; k++) {
		const uint64_t t = b.timestamp_sample + b.dt[k];
		EXPECT_EQ(t - kStartUs, static_cast<uint64_t>(b.x[k]) * kIcmIntervalUs) << "sample " << k;
		EXPECT_EQ(b.y[k], -b.x[k]);
	}
}
}

// An 8 kHz ICM42688P is averaged by 5: each batch sample is the mean of 5 consecutive samples at
// their mean time, 1.6 kHz, 32 to a batch. Its FIFO clock reports 120 us where the samples come every
// 125 us: the measured interval places them.
TEST(FormicImuBatch, EightKilohertzIsAveragedByFive)
{
	SampleBatcher batcher;
	Sink sink;

	for (int m = 0; m < 40; m++) {
		addIcmMessage(batcher, sink, m, 120.f);
	}

	EXPECT_EQ(batcher.factor(), 5);
	EXPECT_NEAR(batcher.inputRateHz(), 8000.f, 1.f);
	ASSERT_EQ(sink.batches.size(), 2u);

	for (int k = 0; k < 32; k++) {
		EXPECT_EQ(sink.batches[0].x[k], 5 * k + 2);
		EXPECT_EQ(sink.batches[0].z[k], 1000);
	}

	// the first message was placed with the reported 120 us; from the second on with the measured 125 us
	const formic_imu_batch_s &b = sink.batches[1];
	EXPECT_EQ(b.samples, 32);
	EXPECT_EQ(b.device_id, kIcmDevice);
	EXPECT_EQ(b.scale, kIcmScale);
	EXPECT_EQ(b.timestamp_sample, kStartUs + 162 * kIcmIntervalUs);

	for (int k = 0; k < 32; k++) {
		EXPECT_EQ(b.dt[k], k * 625);
		EXPECT_EQ(b.x[k], 5 * (32 + k) + 2);
	}

	expectMeansOfConsecutiveSamples(b);
	EXPECT_EQ(batcher.emitted(), 64u);
	EXPECT_EQ(batcher.lost(), 0u);
	EXPECT_EQ(batcher.dropped(), 0u);
}

// A lost FIFO message: the part group before it is dropped, never averaged across the gap, and the
// missing batch samples are counted (inputs 85 to 95: means at 82 and then 98, 3.2 steps apart, 2 lost).
TEST(FormicImuBatch, AGroupNeverSpansAGap)
{
	SampleBatcher batcher;
	Sink sink;

	for (int m = 0; m < 41; m++) {
		if (m != 11) {
			addIcmMessage(batcher, sink, m, 125.f);
		}
	}

	batcher.flush(sink);
	ASSERT_EQ(sink.batches.size(), 2u);
	EXPECT_EQ(sink.batches[0].x[16], 82);
	EXPECT_EQ(sink.batches[0].x[17], 98);
	EXPECT_EQ(sink.batches[0].samples_lost, 2u);
	EXPECT_EQ(batcher.lost(), 2u);
	EXPECT_EQ(batcher.emitted(), 63u);

	for (const formic_imu_batch_s &b : sink.batches) {
		expectMeansOfConsecutiveSamples(b);
	}
}

// The ICM42688P accel switches between 16 bit data (1/8192 g) and 20 bit data scaled to 16 bit
// (1/2048 g) from one FIFO message to the next. A group across the switch is averaged at the coarser
// scale; each batch keeps one scale.
TEST(FormicImuBatch, ScaleChangeInAGroupAveragesAtTheCoarserScale)
{
	SampleBatcher batcher;
	Sink sink;
	constexpr float kFine = 9.80665f / 8192.f;
	constexpr float kCoarse = 9.80665f / 2048.f;

	// the same acceleration in both: 400 * kFine == 100 * kCoarse
	addIcmMessage(batcher, sink, 0, 125.f, kFine, 400, 8000);	// inputs 0-7
	addIcmMessage(batcher, sink, 1, 125.f, kCoarse, 100, 2000);	// inputs 8-15
	addIcmMessage(batcher, sink, 2, 125.f, kFine, 400, 8000);	// inputs 16-23
	addIcmMessage(batcher, sink, 3, 125.f, kFine, 400, 8000);	// inputs 24-31
	batcher.flush(sink);

	// means of 0-4 | 5-9, 10-14, 15-19 | 20-24, 25-29
	ASSERT_EQ(sink.batches.size(), 3u);
	EXPECT_EQ(sink.batches[0].samples, 1);
	EXPECT_EQ(sink.batches[0].scale, kFine);
	EXPECT_EQ(sink.batches[0].x[0], 400);

	const formic_imu_batch_s &coarse = sink.batches[1];
	EXPECT_EQ(coarse.samples, 3);
	EXPECT_EQ(coarse.scale, kCoarse);

	for (int k = 0; k < 3; k++) {
		EXPECT_EQ(coarse.x[k], 100);
		EXPECT_EQ(coarse.y[k], -100);
		EXPECT_EQ(coarse.z[k], 2000);
	}

	EXPECT_EQ(sink.batches[2].samples, 2);
	EXPECT_EQ(sink.batches[2].scale, kFine);
	EXPECT_EQ(sink.batches[2].x[1], 400);
	EXPECT_EQ(batcher.lost(), 0u);
}

// The factor brings a sensor nearest 1.6 kHz: 8 kHz by 5, 3.2 kHz by 2, 1.6 kHz and slower as they are.
// It follows the sensor: a reset (another IMU selected) takes the next sensor's.
TEST(FormicImuBatch, FactorFollowsTheRate)
{
	EXPECT_EQ(SampleBatcher::averagingFactor(125.f), 5);
	EXPECT_EQ(SampleBatcher::averagingFactor(119.47f), 5);	// as the ICM42688P on the bench drone reports
	EXPECT_EQ(SampleBatcher::averagingFactor(312.5f), 2);
	EXPECT_EQ(SampleBatcher::averagingFactor(625.f), 1);
	EXPECT_EQ(SampleBatcher::averagingFactor(1000.f), 1);
	EXPECT_EQ(SampleBatcher::averagingFactor(0.f), 1);
	EXPECT_EQ(SampleBatcher::averagingFactor(NAN), 1);

	SampleBatcher batcher;
	Sink sink;
	EXPECT_EQ(batcher.factor(), 0);
	addMessage(batcher, sink, kStartUs, 1, 1);
	EXPECT_EQ(batcher.factor(), 1);

	batcher.reset();
	addIcmMessage(batcher, sink, 100, 125.f);
	EXPECT_EQ(batcher.factor(), 5);
}

// Samples are placed by the measured interval, not the reported dt: a sensor reporting 600 us with
// samples every 625 us, 4 to a message. A lost message is no measurement and counts its samples.
TEST(FormicImuBatch, TheIntervalIsMeasured)
{
	SampleBatcher batcher;
	Sink sink;
	constexpr uint64_t kIntervalUs = 625;

	auto add4 = [&](int m) {
		int16_t x[4], y[4] {}, z[4] {};

		for (int i = 0; i < 4; i++) {
			x[i] = static_cast<int16_t>(4 * m + i);
		}

		batcher.add(kStartUs + static_cast<uint64_t>(4 * m + 3) * kIntervalUs, 600.f, kDevice, kScale, 4, x, y, z, sink);
	};

	for (int m = 0; m < 24; m++) {
		add4(m);
	}

	EXPECT_EQ(batcher.factor(), 1);
	ASSERT_EQ(sink.batches.size(), 3u);
	const formic_imu_batch_s &b = sink.batches[1];
	EXPECT_EQ(b.timestamp_sample, kStartUs + 32 * kIntervalUs);

	for (int k = 0; k < 32; k++) {
		EXPECT_EQ(b.dt[k], k * kIntervalUs);
		EXPECT_EQ(b.x[k], 32 + k);
	}

	add4(25);	// message 24 lost
	EXPECT_NEAR(batcher.inputRateHz(), 1600.f, 0.01f);
	EXPECT_EQ(batcher.lost(), 4u);
}

// Off until the default is set; then the default's state, set by the parameter.
TEST(FormicBatchGate, StartsInTheDefaultState)
{
	BatchGate on;
	EXPECT_FALSE(on.enabled());
	on.setDefault(true);
	EXPECT_TRUE(on.enabled());
	EXPECT_EQ(on.source(), BatchGate::Source::Parameter);

	BatchGate off;
	off.setDefault(false);
	EXPECT_FALSE(off.enabled());
	EXPECT_FALSE(off.takeChange());
}

// enable: true turns on, enable: false turns off; a repeated command changes nothing.
TEST(FormicBatchGate, CommandTurnsOnAndOff)
{
	BatchGate gate;
	gate.setDefault(false);

	gate.command(true);
	EXPECT_TRUE(gate.enabled());
	EXPECT_EQ(gate.source(), BatchGate::Source::Command);
	EXPECT_TRUE(gate.takeChange());

	gate.command(true);
	EXPECT_TRUE(gate.enabled());
	EXPECT_FALSE(gate.takeChange());

	gate.command(false);
	EXPECT_FALSE(gate.enabled());
	EXPECT_TRUE(gate.takeChange());

	gate.command(false);
	EXPECT_FALSE(gate.enabled());
	EXPECT_FALSE(gate.takeChange());
}

// Disarm returns to the default, for both defaults; arming changes nothing.
TEST(FormicBatchGate, DisarmReturnsToTheDefault)
{
	BatchGate gate;
	gate.setDefault(false);
	gate.setArmed(false);
	gate.command(true);

	gate.setArmed(true);
	EXPECT_TRUE(gate.enabled());
	EXPECT_EQ(gate.source(), BatchGate::Source::Command);

	gate.setArmed(false);
	EXPECT_FALSE(gate.enabled());
	EXPECT_EQ(gate.source(), BatchGate::Source::Disarm);

	BatchGate gate_on;
	gate_on.setDefault(true);
	gate_on.setArmed(true);
	gate_on.command(false);
	EXPECT_FALSE(gate_on.enabled());

	gate_on.setArmed(false);
	EXPECT_TRUE(gate_on.enabled());
	EXPECT_EQ(gate_on.source(), BatchGate::Source::Disarm);

	// arming, and staying disarmed, keep a command
	gate_on.command(false);
	gate_on.setArmed(false);
	EXPECT_FALSE(gate_on.enabled());
	gate_on.setArmed(true);
	EXPECT_FALSE(gate_on.enabled());
}

// The first arming state only records it: a module started on a disarmed vehicle sees no disarm.
TEST(FormicBatchGate, FirstArmingStateIsNoDisarm)
{
	BatchGate gate;
	gate.setDefault(false);
	gate.command(true);
	gate.setArmed(false);
	EXPECT_TRUE(gate.enabled());
}

// A new default applies at once and overrides an earlier command; the same default again does not.
TEST(FormicBatchGate, ParameterChangeAppliesAtOnce)
{
	BatchGate gate;
	gate.setDefault(true);
	gate.command(false);

	gate.setDefault(true);
	EXPECT_FALSE(gate.enabled());

	gate.setDefault(false);
	gate.command(true);
	EXPECT_TRUE(gate.enabled());

	gate.setDefault(true);
	EXPECT_TRUE(gate.enabled());

	gate.command(false);
	gate.setDefault(false);
	gate.setDefault(true);
	EXPECT_TRUE(gate.enabled());
	EXPECT_EQ(gate.source(), BatchGate::Source::Parameter);
}

// A change of state is reported exactly once; off and on again before it is read is none.
TEST(FormicBatchGate, ChangeIsReportedOnce)
{
	BatchGate gate;
	gate.setDefault(true);
	EXPECT_TRUE(gate.takeChange());
	EXPECT_FALSE(gate.takeChange());

	gate.command(false);
	EXPECT_TRUE(gate.takeChange());
	EXPECT_FALSE(gate.takeChange());

	gate.command(true);
	gate.command(false);
	EXPECT_FALSE(gate.takeChange());
	EXPECT_FALSE(gate.enabled());
}
