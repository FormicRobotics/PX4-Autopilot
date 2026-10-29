# One camera frame as the companion computer saw it, for the strobe-based clock sync.
# The companion publishes it on /fmu/in/camera_frame_stamp once per delivered frame.
# The uXRCE-DDS client converts `timestamp` to the flight controller clock on receive;
# every other field stays as the companion wrote it.

uint64 timestamp          # publish time on the companion clock, microseconds since epoch (converted on receive)
uint64 frame_realtime_ns  # start-of-frame time on the companion CLOCK_REALTIME, ns. The clock the DDS agent syncs against.
uint64 frame_monotonic_ns # start-of-frame time on the companion CLOCK_MONOTONIC, ns. The raw V4L2 buffer stamp.
uint32 frame_counter      # frame index: +1 per frame, plus frames the driver dropped (from the stamp interval), so strobe_seq - frame_counter stays constant. Restarts with the publisher.
uint32 driver_sequence    # V4L2 buffer sequence number of the frame
uint8 camera_id           # which camera, for setups with more than one strobe source
