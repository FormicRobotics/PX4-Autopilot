uint64 timestamp      # time since system start (microseconds)

# NED local frame, same convention as TrajectorySetpoint.
float32 vx             # commanded north velocity, m/s
float32 vy             # commanded east velocity, m/s
float32 vz             # commanded down velocity, m/s, +down
float32 psi_dot_cmd    # commanded yaw rate, rad/s, +CW
