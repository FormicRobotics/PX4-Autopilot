uint64 timestamp	# time since system start (microseconds)
uint64 timestamp_utc # UTC timestamp

uint32 seq		# Image sequence number
bool feedback	# Trigger feedback from camera

uint32 ORB_QUEUE_LENGTH = 2

# camera_trigger: the camera_capture strobe (ring cameras) or the camera_trigger driver's trigger.
# tracker_strobe: the tracker camera strobe, from formic_strobe_capture.
# TOPICS camera_trigger tracker_strobe
