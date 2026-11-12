px4_add_module(
	MODULE modules__formic_vis_servo
	MAIN formic_vis_servo
	SRCS
		formic_vis_servo.cpp
	DEPENDS
		px4_work_queue
		mathlib
)
