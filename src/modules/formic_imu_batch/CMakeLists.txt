px4_add_module(
	MODULE modules__formic_imu_batch
	MAIN formic_imu_batch
	SRCS
		FormicImuBatch.cpp
		FormicImuBatch.hpp
		SampleBatcher.hpp
		BatchGate.hpp
	DEPENDS
		px4_work_queue
)

px4_add_unit_gtest(SRC FormicImuBatchTest.cpp LINKLIBS uorb_msgs)
