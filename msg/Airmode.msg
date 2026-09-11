# Airmode status published by manual_control
#
# Mirrors the MC_AIRMODE param handling done internally by the manual_control module,
# so consumers (e.g. mixer/attitude control) can react to a ground-contact forced-off
# in real time instead of waiting on a param_set() round trip.

uint64 timestamp # time since system start (microseconds)

uint8 user_airmode      # the user's configured MC_AIRMODE param value (0: off, 1: roll/pitch, 2: roll/pitch/yaw)
bool airmode_forced_off # true if airmode is currently forced off due to ground contact / ground effect
