px4_add_module(
	MODULE modules__camera_timesync
	MAIN camera_timesync
	SRCS
		camera_timesync.cpp
	DEPENDS
		px4_work_queue
	)
