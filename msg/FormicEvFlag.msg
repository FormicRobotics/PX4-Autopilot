uint64 timestamp                          # time since system start (microseconds)


bool ev_data_arrived   #when have ev data that arrived from the dds
bool ekfs_converged   # when the EKF yaw and position has converged
bool heading_ok   # when the heading is aligned with the EV (used to count resets)
bool pos_ok   # when the position is aligned with the EV (used to count resets)

# Dstate Enum Constants
uint8 dstate
