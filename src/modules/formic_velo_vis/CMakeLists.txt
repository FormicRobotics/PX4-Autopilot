px4_add_module(
	MODULE modules__formic_velo_vis
	MAIN formic_velo_vis
	SRCS
		formic_velo_vis.cpp
	DEPENDS
		px4_work_queue
		mathlib
)
