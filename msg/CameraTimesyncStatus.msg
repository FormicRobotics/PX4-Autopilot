# State of the strobe-based clock sync between the companion computer and the flight controller.
# Published by the camera_timesync module once per matched frame and at least every 500 ms.
# Offsets follow the Timesync convention: fc_time_us = companion_time_us + offset.

uint64 timestamp                # time since system start (microseconds)

uint8 STATE_NO_TRIGGERS = 0     # no camera_trigger samples yet (camera_capture off or strobe not wired)
uint8 STATE_NO_FRAMES = 1       # strobes arrive but no camera_frame_stamp from the companion
uint8 STATE_WAITING_DDS = 2     # cannot match: no coarse offset yet (DDS timesync not converged)
uint8 STATE_ACQUIRING = 3       # matching, collecting the first samples
uint8 STATE_LOCKED = 4          # estimate valid
uint8 STATE_STALE = 5           # was locked, no accepted sample for CAMTS_TIMEOUT
uint8 state

bool apply                      # true when the uxrce_dds_client uses estimated_offset for its time offset

uint32 frame_counter            # of the last matched frame
uint32 driver_sequence          # V4L2 sequence of the last matched frame
uint32 strobe_seq               # camera_trigger.seq matched to it
uint64 frame_realtime_ns        # companion stamp of the matched frame (ns, companion CLOCK_REALTIME)
uint64 strobe_timestamp         # hrt time of the matched strobe edge (microseconds, not converted)
int64 observed_offset           # strobe_timestamp - frame_realtime (us): one raw measurement
int64 estimated_offset          # filtered offset (us) at `timestamp`, the value the DDS client applies
float32 drift_ppm               # rate of estimated_offset (us per s): how fast the companion clock runs against the FC clock
int64 dds_offset                # estimated_offset of the DDS round-trip timesync (us), for comparison
int64 match_error               # strobe_timestamp - (frame_realtime + hint offset) (us) at matching time
uint32 strobe_to_receive        # hrt now - strobe_timestamp when the frame stamp arrived (us)
uint32 transport_latency        # hrt now - converted companion publish time when the frame stamp arrived (us)
uint32 strobe_period            # median interval between strobes (us)
uint32 match_window             # half-width of the accept window used for matching (us)
uint32 matched_count            # frames matched to a strobe since start
uint32 rejected_count           # matched frames whose offset was too far from the estimate
uint32 unmatched_count          # frames with no strobe inside the window
uint32 counter_mismatch_count   # matched frames whose frame_counter step disagrees with the strobe periods elapsed since the previous match: a frame lost without a stamp gap (the companion skips its counter over driver drops). Not counted across a companion restart.
uint32 strobe_seq_error_count   # matched strobes whose camera_trigger.seq step disagrees with the strobe periods elapsed: an edge counted with no strobe behind it (bounce on the strobe line) or a missed edge
