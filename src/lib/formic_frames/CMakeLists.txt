# Header-only helpers for the Formic VIO pose, shared by formic_watchdog_ev and formic_vio_est.

px4_add_unit_gtest(SRC FormicSampleTimeTest.cpp)
px4_add_unit_gtest(SRC FormicBoardFrameTest.cpp)
px4_add_unit_gtest(SRC FormicAnchorTest.cpp)
px4_add_unit_gtest(SRC FormicControlStateTest.cpp LINKLIBS uorb_msgs)
