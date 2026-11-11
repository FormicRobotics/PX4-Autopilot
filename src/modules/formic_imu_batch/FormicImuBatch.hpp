#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/formic_imu_batch.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/sensor_selection.h>

#include "SampleBatcher.hpp"

using namespace time_literals;

/**
 * Every FIFO sample of the IMU EKF2 flies on, for the Jetson.
 *
 * Follows sensor_selection to the gyro and accel in use, reads their sensor_gyro_fifo and
 * sensor_accel_fifo on every publication and publishes the samples in batches on
 * formic_gyro_batch and formic_accel_batch. Each batch carries the sensor temperature from the
 * newest sensor_gyro / sensor_accel of its device.
 */
class FormicImuBatch : public ModuleBase<FormicImuBatch>, public px4::ScheduledWorkItem
{
public:
	FormicImuBatch();
	~FormicImuBatch() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	static constexpr int MAX_INSTANCES{4};
	static constexpr int NONE{-1};

	void Run() override;

	/// Follows sensor_selection: the FIFO and report instances of the gyro and accel in use.
	void updateSelection();

	/// Publishes a batch, stamped now, with `temperature` (degrees Celsius, NaN when unknown).
	void publish(uORB::Publication<formic_imu_batch_s> &pub, const formic_imu_batch_s &batch, float temperature);

	/// FIFO messages uORB dropped before this module read them (its queue overflowed): the jumps in the
	/// subscription's generation. The first read after a change of instance is not counted.
	static void countOverflow(unsigned generation_before, unsigned generation_after, bool &first, uint32_t &overflow);

	/// The instance of topic T (a FIFO or report topic) whose device_id is `device_id`, or NONE.
	template<typename T, typename S>
	static int findInstance(S (&subs)[MAX_INSTANCES], uint32_t device_id);

	/// The temperature in the newest report T (sensor_gyro or sensor_accel) on `instance`, for a batch
	/// of `device_id` (degrees Celsius); NaN when there is none.
	template<typename T>
	static float reportTemperature(uORB::Subscription (&subs)[MAX_INSTANCES], int instance, uint32_t device_id);

	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};

	uORB::SubscriptionCallbackWorkItem _gyro_fifo_sub[MAX_INSTANCES] {
		{this, ORB_ID(sensor_gyro_fifo), 0},
		{this, ORB_ID(sensor_gyro_fifo), 1},
		{this, ORB_ID(sensor_gyro_fifo), 2},
		{this, ORB_ID(sensor_gyro_fifo), 3},
	};

	uORB::SubscriptionCallbackWorkItem _accel_fifo_sub[MAX_INSTANCES] {
		{this, ORB_ID(sensor_accel_fifo), 0},
		{this, ORB_ID(sensor_accel_fifo), 1},
		{this, ORB_ID(sensor_accel_fifo), 2},
		{this, ORB_ID(sensor_accel_fifo), 3},
	};

	// the driver publishes these next to each FIFO message, with the sensor temperature
	uORB::Subscription _gyro_sub[MAX_INSTANCES] {
		{ORB_ID(sensor_gyro), 0},
		{ORB_ID(sensor_gyro), 1},
		{ORB_ID(sensor_gyro), 2},
		{ORB_ID(sensor_gyro), 3},
	};

	uORB::Subscription _accel_sub[MAX_INSTANCES] {
		{ORB_ID(sensor_accel), 0},
		{ORB_ID(sensor_accel), 1},
		{ORB_ID(sensor_accel), 2},
		{ORB_ID(sensor_accel), 3},
	};

	uORB::Publication<formic_imu_batch_s> _gyro_batch_pub{ORB_ID(formic_gyro_batch)};
	uORB::Publication<formic_imu_batch_s> _accel_batch_pub{ORB_ID(formic_accel_batch)};

	formic_imu_batch::SampleBatcher _gyro_batcher{};
	formic_imu_batch::SampleBatcher _accel_batcher{};

	uint32_t _gyro_device_id{0};
	uint32_t _accel_device_id{0};
	int _gyro_instance{NONE};
	int _accel_instance{NONE};
	int _gyro_report_instance{NONE};
	int _accel_report_instance{NONE};
	hrt_abstime _last_selection_check{0};

	// The gyro and accel batches would fill on the same sample and reach the link back to back. The
	// accel batch is closed when the gyro batch is half full, so the two alternate every 16 samples.
	static constexpr uint8_t STAGGER_SAMPLES{formic_imu_batch_s::MAX_SAMPLES / 2};
	static constexpr uint8_t STAGGER_MIN_SAMPLES{8}; // a shorter accel batch is left to fill
	bool _accel_stagger_armed{false};

	bool _gyro_first_read{true};
	bool _accel_first_read{true};
	uint32_t _gyro_queue_overflow{0};
	uint32_t _accel_queue_overflow{0};

	// read buffers: kept off the work queue's stack
	sensor_gyro_fifo_s _gyro_fifo{};
	sensor_accel_fifo_s _accel_fifo{};
	formic_imu_batch_s _out{};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
};
