uint64 timestamp      # time since system start (microseconds)

# Limits formic_vis_servo applies to formic_vis_servo_setpoint (FMTRK_* params),
# published so the off-board controller can plan against the same bounds.
float32 tilt_max       # |phi_cmd|, |theta_cmd| clamp, rad (FMTRK_TILT_MAX)
float32 vz_max         # |v_cmd| clamp, m/s (FMTRK_VZ_MAX)
