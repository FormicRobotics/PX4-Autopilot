# One target bounding box from the companion computer's vision tracker, published on
# /fmu/in/formic_target_box once per frame the tracker locked onto. A frame without a lock sends
# nothing, so a consumer tells "target lost" from the age of `timestamp`, the way the Formic
# modes do with their loss timeouts.
#
# The uXRCE-DDS client converts `timestamp` to the flight controller clock on receive; every
# other field stays as the companion wrote it. `frame_realtime_ns`, `driver_sequence` and
# `camera_id` are that same frame's fields in CameraFrameStamp, so the box joins to the frame
# camera_timesync matched against the strobe: that match, not `timestamp`, is the box's exact
# capture time.

uint64 timestamp           # publish time on the companion clock, microseconds since epoch (converted on receive)
uint64 frame_realtime_ns   # start-of-frame time of the frame the box was found in, companion CLOCK_REALTIME, ns
uint64 tracker_realtime_ns # when the detector finished that frame, companion CLOCK_REALTIME, ns: minus frame_realtime_ns is the vision latency

uint32 driver_sequence     # V4L2 buffer sequence number of that frame

float32 box_x              # bounding box top-left corner in the full frame, pixels, +x right
float32 box_y              # bounding box top-left corner in the full frame, pixels, +y down
float32 box_width          # bounding box width, pixels
float32 box_height         # bounding box height, pixels
float32 confidence         # detector score for the box, 0..1

uint16 frame_width         # size of the image the box is in, pixels
uint16 frame_height        # size of the image the box is in, pixels

uint8 camera_id            # which camera, as in CameraFrameStamp
uint8 target_color         # first letter of the target colour ('r', 'b', 'g', 'y'), 'u' if unset
uint8 target_id            # which target, for a tracker that follows more than one
