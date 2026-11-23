uint64 timestamp                          # time since system start (microseconds)

bool pos_req          # a mode that needs position is flown or wanted
bool use_vision_now   # a position source should fly now: the EKF2 selector publishes the source its arbiter picks (the VIO instance or EKF2 on GPS)

uint8 SRC_RC_NONE = 0 # no RC source switch (FORMIC_SRC_RC 0, or no RC): FORMIC_EST_PREF decides
uint8 SRC_RC_VIO = 1  # switch low: VIO only
uint8 SRC_RC_AUTO = 2 # switch middle: FORMIC_EST_PREF decides
uint8 SRC_RC_GPS = 3  # switch high: GPS only
uint8 src_rc          # position of the RC source switch
