px4_add_module(
	MODULE modules__formic_tracker
	MAIN formic_tracker
	SRCS
		formic_tracker.cpp
	DEPENDS
		px4_work_queue
		mathlib
)
