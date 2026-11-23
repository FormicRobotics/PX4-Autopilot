/****************************************************************************
 * The position source of a mode that needs position: the VIO (formic_vio_est, estimator instance 1)
 * or GPS (EKF2, estimator instance 0).
 *
 * The arbiter reads status only. No data moves from one estimator into the other. There is no uORB
 * in here, so it is tested alone. The EKF2 selector runs it every cycle.
 ****************************************************************************/

#pragma once

#include <cmath>
#include <cstdint>

namespace formic_frames
{

enum class Source : uint8_t {
	NONE = 0,  ///< no source is good: Altitude mode
	VIO = 1,
	GPS = 2,
};

/// FORMIC_EST_PREF
enum class SourcePref : uint8_t {
	AUTO_VIO_FIRST = 0,
	AUTO_GPS_FIRST = 1,
	VIO_ONLY = 2,
	GPS_ONLY = 3,
};

/// Position of the RC source switch (FORMIC_SRC_RC). It wins over FORMIC_EST_PREF.
enum class SourceRc : uint8_t {
	NONE = 0,  ///< no switch: the parameter
	VIO = 1,   ///< low: VIO only
	AUTO = 2,  ///< middle: the parameter
	GPS = 3,   ///< high: GPS only
};

/// Why GPS is not good, one bit per check.
namespace gps_fail
{
static constexpr uint16_t STALE = 1 << 0;   ///< no GPS message
static constexpr uint16_t FIX = 1 << 1;     ///< less than a 3D fix
static constexpr uint16_t SAT = 1 << 2;     ///< satellites
static constexpr uint16_t HDOP = 1 << 3;
static constexpr uint16_t EPH = 1 << 4;     ///< horizontal accuracy
static constexpr uint16_t SACC = 1 << 5;    ///< speed accuracy
static constexpr uint16_t JAM = 1 << 6;
static constexpr uint16_t SPOOF = 1 << 7;
static constexpr uint16_t EKF = 1 << 8;     ///< EKF2 does not fuse GPS, or its position is not valid
static constexpr uint16_t SPEED = 1 << 9;   ///< its speed differs from the speed of the source flown

/// These turn GPS off at once.
static constexpr uint16_t HARD = STALE | FIX | JAM | SPOOF;
}

struct GpsSample {
	bool fresh{false};
	uint8_t fix_type{0};
	uint8_t satellites{0};
	float hdop{NAN};
	float eph{NAN};
	float speed_acc{NAN};
	bool jammed{false};
	bool spoofed{false};
	bool ekf_fusing{false};    ///< EKF2 fuses GPS position
	bool ekf_xy_valid{false};  ///< EKF2's local position is valid
};

struct ArbiterLimits {
	uint8_t sat_min{10};
	float hdop_max{1.5f};
	float eph_max{2.f};
	float speed_acc_max{0.5f};
	uint64_t t_good_us{5'000'000};     ///< every check passed for this long: GPS good
	uint64_t t_bad_us{1'000'000};      ///< a check failed for this long: GPS not good
	uint64_t dwell_us{3'000'000};      ///< no switch back for this long, unless the source flown fails
	uint8_t sat_margin{2};             ///< to turn off, satellites and HDOP pass the limit by this much
	float hdop_margin{0.5f};
	float speed_diff_max{1.f};         ///< horizontal speeds of the two sources agree within this, m/s
	float speed_ratio_max{0.2f};       ///< and, moving, within this part of the speed
	float speed_moving{2.f};           ///< "moving", m/s
};

struct ArbiterInput {
	uint64_t now_us{0};
	GpsSample gps{};
	bool vio_enter{false};     ///< the VIO may be switched to
	bool vio_stay{false};      ///< the VIO may carry on
	bool speeds_valid{false};  ///< both speeds below are usable
	float gps_speed{0.f};      ///< horizontal speed of EKF2, m/s
	float vio_speed{0.f};      ///< horizontal speed of the VIO instance, m/s
	SourcePref pref{SourcePref::AUTO_VIO_FIRST};
	SourceRc rc{SourceRc::NONE};
};

struct ArbiterOutput {
	Source source{Source::NONE};
	bool gps_good{false};
	bool vio_good{false};
	uint16_t gps_fail{0};          ///< gps_fail bits
	bool vio_speed_blocked{false}; ///< the VIO is not switched to: its speed differs from GPS
	SourcePref setting{SourcePref::AUTO_VIO_FIRST};  ///< the parameter after the RC switch
};

/// The parameter after the RC switch.
inline SourcePref effective_setting(SourcePref pref, SourceRc rc)
{
	switch (rc) {
	case SourceRc::VIO: return SourcePref::VIO_ONLY;

	case SourceRc::GPS: return SourcePref::GPS_ONLY;

	default: return pref;
	}
}

/// The decision table.
inline Source pick_source(SourcePref setting, bool gps_good, bool vio_good)
{
	switch (setting) {
	case SourcePref::AUTO_GPS_FIRST:
		return gps_good ? Source::GPS : (vio_good ? Source::VIO : Source::NONE);

	case SourcePref::VIO_ONLY:
		return vio_good ? Source::VIO : Source::NONE;

	case SourcePref::GPS_ONLY:
		return gps_good ? Source::GPS : Source::NONE;

	case SourcePref::AUTO_VIO_FIRST:
	default:
		return vio_good ? Source::VIO : (gps_good ? Source::GPS : Source::NONE);
	}
}

/**
 * The GPS checks of one message.
 * @param margin true: the limits to turn off (satellites and HDOP pass the limit by the margin)
 */
inline uint16_t gps_checks(const GpsSample &gps, const ArbiterLimits &lim, bool margin)
{
	uint16_t fail = 0;

	if (!gps.fresh) {
		// nothing else can be judged
		return gps_fail::STALE;
	}

	if (gps.fix_type < 3) {
		fail |= gps_fail::FIX;
	}

	const int sat_min = static_cast<int>(lim.sat_min) - (margin ? static_cast<int>(lim.sat_margin) : 0);
	const float hdop_max = lim.hdop_max + (margin ? lim.hdop_margin : 0.f);

	if (static_cast<int>(gps.satellites) < sat_min) {
		fail |= gps_fail::SAT;
	}

	// a value that is not finite fails
	if (!(gps.hdop <= hdop_max)) {
		fail |= gps_fail::HDOP;
	}

	if (!(gps.eph <= lim.eph_max)) {
		fail |= gps_fail::EPH;
	}

	if (!(gps.speed_acc <= lim.speed_acc_max)) {
		fail |= gps_fail::SACC;
	}

	if (gps.jammed) {
		fail |= gps_fail::JAM;
	}

	if (gps.spoofed) {
		fail |= gps_fail::SPOOF;
	}

	if (!gps.ekf_fusing || !gps.ekf_xy_valid) {
		fail |= gps_fail::EKF;
	}

	return fail;
}

class SourceArbiter
{
public:
	ArbiterLimits limits{};

	Source source() const { return _source; }

	ArbiterOutput update(const ArbiterInput &in)
	{
		ArbiterOutput out;
		out.setting = effective_setting(in.pref, in.rc);

		const bool gps_quality = updateGps(in, out.gps_fail);

		// The speeds are compared in size only, so no common frame is needed. A difference keeps the source that
		// is not flown out. It never removes the one that is flown: that one has its own checks.
		const bool vio_any = (_source == Source::VIO) ? in.vio_stay : in.vio_enter;
		bool mismatch = false;

		if (in.speeds_valid && gps_quality && vio_any) {
			const float diff = fabsf(in.gps_speed - in.vio_speed);
			mismatch = (diff > limits.speed_diff_max)
				   || ((in.gps_speed > limits.speed_moving) && (diff > limits.speed_ratio_max * in.gps_speed));
		}

		const bool vio_first = (out.setting == SourcePref::AUTO_VIO_FIRST) || (out.setting == SourcePref::VIO_ONLY);
		const bool gps_blocked = mismatch && ((_source == Source::VIO) || ((_source == Source::NONE) && vio_first));
		const bool vio_blocked = mismatch && ((_source == Source::GPS) || ((_source == Source::NONE) && !vio_first));

		if (gps_blocked) {
			out.gps_fail |= gps_fail::SPEED;
		}

		out.gps_good = gps_quality && !gps_blocked;
		out.vio_good = vio_any && !vio_blocked;
		out.vio_speed_blocked = vio_any && vio_blocked;

		const Source wanted = pick_source(out.setting, out.gps_good, out.vio_good);

		if (wanted != _source) {
			const bool current_good = (_source == Source::GPS) ? out.gps_good
						  : ((_source == Source::VIO) ? out.vio_good : false);
			const bool setting_changed = !_setting_known || (out.setting != _setting);
			const bool dwell_over = (in.now_us >= _last_change_us + limits.dwell_us);

			// a good source is left for the preferred one only after the dwell time; the pilot's choice acts at once
			if (!current_good || setting_changed || dwell_over) {
				_source = wanted;
				_last_change_us = in.now_us;
			}
		}

		_setting = out.setting;
		_setting_known = true;
		out.source = _source;
		return out;
	}

private:
	bool updateGps(const ArbiterInput &in, uint16_t &fail_out)
	{
		const uint16_t fail_on = gps_checks(in.gps, limits, false);
		const uint16_t fail_off = gps_checks(in.gps, limits, true);

		if (!_gps_good) {
			_gps_fail_since_us = 0;

			if (fail_on != 0) {
				_gps_pass_since_us = 0;

			} else {
				if (_gps_pass_since_us == 0) {
					_gps_pass_since_us = in.now_us;
				}

				if (in.now_us >= _gps_pass_since_us + limits.t_good_us) {
					_gps_good = true;
				}
			}

		} else {
			_gps_pass_since_us = 0;

			if ((fail_off & gps_fail::HARD) != 0) {
				_gps_good = false;

			} else if (fail_off != 0) {
				if (_gps_fail_since_us == 0) {
					_gps_fail_since_us = in.now_us;
				}

				if (in.now_us >= _gps_fail_since_us + limits.t_bad_us) {
					_gps_good = false;
				}

			} else {
				_gps_fail_since_us = 0;
			}
		}

		// good: what is failing towards off. Not good: what keeps it from turning on.
		fail_out = _gps_good ? fail_off : fail_on;
		return _gps_good;
	}

	Source _source{Source::NONE};
	uint64_t _last_change_us{0};
	SourcePref _setting{SourcePref::AUTO_VIO_FIRST};
	bool _setting_known{false};

	bool _gps_good{false};
	uint64_t _gps_pass_since_us{0};
	uint64_t _gps_fail_since_us{0};
};

} // namespace formic_frames
