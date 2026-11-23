# The position source of a mode that needs position, picked by the source arbiter in the EKF2 selector:
# the VIO (formic_vio_est, estimator instance 1) or GPS (EKF2, estimator instance 0).

uint64 timestamp			# time since system start (microseconds)

uint8 SOURCE_NONE = 0			# no source is good: Altitude mode
uint8 SOURCE_VIO = 1
uint8 SOURCE_GPS = 2
uint8 source				# the source a mode that needs position flies on now
bool source_active			# the primary estimator instance is this source: the mode can be entered

bool gps_good				# every GPS check passed for FORMIC_GPS_TGOOD; off after FORMIC_GPS_TBAD of a failed check
bool vio_good				# the VIO may be switched to, or may carry on when it is the source
bool vio_speed_blocked			# the VIO is not switched to: its speed differs from GPS

uint16 GPS_FAIL_STALE = 1		# no GPS message
uint16 GPS_FAIL_FIX = 2			# less than a 3D fix
uint16 GPS_FAIL_SAT = 4			# satellites under FORMIC_GPS_SAT
uint16 GPS_FAIL_HDOP = 8		# HDOP over FORMIC_GPS_HDOP
uint16 GPS_FAIL_EPH = 16		# horizontal accuracy over FORMIC_GPS_EPH
uint16 GPS_FAIL_SACC = 32		# speed accuracy over 0.5 m/s
uint16 GPS_FAIL_JAM = 64		# the receiver reports jamming
uint16 GPS_FAIL_SPOOF = 128		# the receiver reports spoofing
uint16 GPS_FAIL_EKF = 256		# EKF2 does not fuse GPS, or its position is not valid
uint16 GPS_FAIL_SPEED = 512		# its speed differs from the speed of the VIO
uint16 gps_fail				# the GPS checks that fail now

uint8 SETTING_AUTO_VIO_FIRST = 0
uint8 SETTING_AUTO_GPS_FIRST = 1
uint8 SETTING_VIO_ONLY = 2
uint8 SETTING_GPS_ONLY = 3
uint8 setting				# FORMIC_EST_PREF after the RC switch
uint8 rc_position			# formic_pos_req.src_rc as used

bool gps_ready				# GPS good and EKF2's heading aligned
bool origin_fitted			# the published frame is EKF2's north with a fixed origin (set at the first gps_ready)

float32 gps_speed			# horizontal speed of EKF2 (m/s); NaN when not valid
float32 vio_speed			# horizontal speed of the VIO instance (m/s); NaN when not valid
