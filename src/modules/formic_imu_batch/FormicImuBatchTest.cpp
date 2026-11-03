#include <gtest/gtest.h>

#include <vector>

#include "SampleBatcher.hpp"

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
