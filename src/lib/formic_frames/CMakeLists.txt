# Header-only helpers for the Formic VIO pose, used by formic_vio_est, the EKF2 selector and the OSD.

px4_add_unit_gtest(SRC FormicSampleTimeTest.cpp)
px4_add_unit_gtest(SRC FormicBoardFrameTest.cpp)
px4_add_unit_gtest(SRC FormicAnchorTest.cpp)
px4_add_unit_gtest(SRC FormicControlStateTest.cpp LINKLIBS uorb_msgs)
px4_add_unit_gtest(SRC FormicPositionNeedTest.cpp LINKLIBS uorb_msgs)
