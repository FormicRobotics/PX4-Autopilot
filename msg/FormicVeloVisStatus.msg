uint64 timestamp      # time since system start (microseconds)

# Limit formic_velo_vis applies to formic_velo_vis_setpoint (FVV_VEL_MAX),
# published so the off-board controller can plan against the same bound.
float32 vel_max        # |vx|, |vy|, |vz| clamp, m/s (FVV_VEL_MAX)
