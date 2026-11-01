px4_add_module(
	MODULE modules__formic_vio_est
	MAIN formic_vio_est
	SRCS
		FormicVioEst.cpp
		FormicVioEst.hpp
		OutputPropagator.hpp
		VioSegment.hpp
	DEPENDS
		px4_work_queue
		sensor_calibration
)

px4_add_unit_gtest(SRC FormicOutputPropagatorTest.cpp)
