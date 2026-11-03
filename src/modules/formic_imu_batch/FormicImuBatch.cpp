#include "FormicImuBatch.hpp"

FormicImuBatch::FormicImuBatch() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

FormicImuBatch::~FormicImuBatch()
{
	for (int i = 0; i < MAX_INSTANCES; i++) {
		_gyro_fifo_sub[i].unregisterCallback();
		_accel_fifo_sub[i].unregisterCallback();
	}

	perf_free(_cycle_perf);
}

bool FormicImuBatch::init()
{
	// the selection is checked on this backup schedule; the samples arrive by callback
	ScheduleOnInterval(100_ms);
	return true;
}

template<typename T>
int FormicImuBatch::findInstance(uORB::SubscriptionCallbackWorkItem (&subs)[MAX_INSTANCES], uint32_t device_id)
{
	if (device_id == 0) {
		return NONE;
	}

	for (int i = 0; i < MAX_INSTANCES; i++) {
		T fifo;

		if (subs[i].copy(&fifo) && (fifo.device_id == device_id)) {
			return i;
		}
	}

	return NONE;
}

void FormicImuBatch::updateSelection()
{
	sensor_selection_s selection;

	if (_sensor_selection_sub.copy(&selection)) {
		if (selection.gyro_device_id != _gyro_device_id) {
			_gyro_device_id = selection.gyro_device_id;
			_gyro_instance = NONE;
		}

		if (selection.accel_device_id != _accel_device_id) {
			_accel_device_id = selection.accel_device_id;
			_accel_instance = NONE;
		}
	}

	// a FIFO instance appears when its driver first publishes: keep looking until both are found
	if (_gyro_instance == NONE) {
		const int i = findInstance<sensor_gyro_fifo_s>(_gyro_fifo_sub, _gyro_device_id);

		for (int k = 0; k < MAX_INSTANCES; k++) {
			_gyro_fifo_sub[k].unregisterCallback();
		}

		if (i != NONE) {
			_gyro_fifo_sub[i].registerCallback();
			_gyro_batcher.reset();
			_gyro_instance = i;
			PX4_INFO("gyro %" PRIu32 ": sensor_gyro_fifo instance %d", _gyro_device_id, i);
		}
	}

	if (_accel_instance == NONE) {
		const int i = findInstance<sensor_accel_fifo_s>(_accel_fifo_sub, _accel_device_id);

		for (int k = 0; k < MAX_INSTANCES; k++) {
			_accel_fifo_sub[k].unregisterCallback();
		}

		if (i != NONE) {
			_accel_fifo_sub[i].registerCallback();
			_accel_batcher.reset();
			_accel_instance = i;
			PX4_INFO("accel %" PRIu32 ": sensor_accel_fifo instance %d", _accel_device_id, i);
		}
	}
}

void FormicImuBatch::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	const hrt_abstime now = hrt_absolute_time();

	if ((now - _last_selection_check) > 100_ms) {
		_last_selection_check = now;
		updateSelection();
	}

	// drain every queued FIFO message, so no sample is skipped while this item was late
	if (_gyro_instance != NONE) {
		sensor_gyro_fifo_s fifo;

		while (_gyro_fifo_sub[_gyro_instance].update(&fifo)) {
			_gyro_batcher.add(fifo.timestamp_sample, fifo.dt, fifo.device_id, fifo.scale, fifo.samples, fifo.x, fifo.y,
					  fifo.z, [this](const formic_imu_batch_s & batch) {
				formic_imu_batch_s out = batch;
				out.timestamp = hrt_absolute_time();
				_gyro_batch_pub.publish(out);
			});
		}
	}

	if (_accel_instance != NONE) {
		sensor_accel_fifo_s fifo;

		while (_accel_fifo_sub[_accel_instance].update(&fifo)) {
			_accel_batcher.add(fifo.timestamp_sample, fifo.dt, fifo.device_id, fifo.scale, fifo.samples, fifo.x, fifo.y,
					   fifo.z, [this](const formic_imu_batch_s & batch) {
				formic_imu_batch_s out = batch;
				out.timestamp = hrt_absolute_time();
				_accel_batch_pub.publish(out);
			});
		}
	}

	perf_end(_cycle_perf);
}

int FormicImuBatch::print_status()
{
	PX4_INFO("gyro %" PRIu32 " (sensor_gyro_fifo instance %d): %" PRIu64 " samples sent, %" PRIu32 " dropped",
		 _gyro_device_id, _gyro_instance, _gyro_batcher.emitted(), _gyro_batcher.dropped());
	PX4_INFO("accel %" PRIu32 " (sensor_accel_fifo instance %d): %" PRIu64 " samples sent, %" PRIu32 " dropped",
		 _accel_device_id, _accel_instance, _accel_batcher.emitted(), _accel_batcher.dropped());
	perf_print_counter(_cycle_perf);
	return 0;
}

int FormicImuBatch::task_spawn(int argc, char *argv[])
{
	FormicImuBatch *instance = new FormicImuBatch();

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int FormicImuBatch::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FormicImuBatch::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Every FIFO sample of the IMU EKF2 flies on, for the Jetson. Follows sensor_selection to the gyro and
accel in use and publishes their sensor_gyro_fifo / sensor_accel_fifo samples in batches of up to 32
on formic_gyro_batch and formic_accel_batch, each sample with its own time.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("formic_imu_batch", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int formic_imu_batch_main(int argc, char *argv[])
{
	return FormicImuBatch::main(argc, argv);
}
