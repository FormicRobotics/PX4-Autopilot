/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef FORMIC_EV_FLAG_HPP
#define FORMIC_EV_FLAG_HPP

#include <uORB/topics/formic_ev_flag.h>

// Sent as NAMED_VALUE_INT "FEV_FLAG":
// bit0 ev_data_arrived, bit1 ekfs_converged, bit2 heading_ok, bit3 pos_ok, bits8-15 dstate
class MavlinkStreamFormicEvFlag : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamFormicEvFlag(mavlink); }

	static constexpr const char *get_name_static() { return "FORMIC_EV_FLAG"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_NAMED_VALUE_INT; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _formic_ev_flag_sub.advertised() ? MAVLINK_MSG_ID_NAMED_VALUE_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamFormicEvFlag(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _formic_ev_flag_sub{ORB_ID(formic_ev_flag)};
	int32_t _last_value{-1};

	bool send() override
	{
		formic_ev_flag_s flag;

		if (_formic_ev_flag_sub.copy(&flag)) {
			mavlink_named_value_int_t msg{};

			// only VisionON (1) and DEGRADED (2) are reported, DR (3) is sent as NONE (0)
			const uint8_t dstate = (flag.dstate == 1 || flag.dstate == 2) ? flag.dstate : 0;

			msg.time_boot_ms = flag.timestamp / 1000ULL;
			strncpy(msg.name, "FEV_FLAG", sizeof(msg.name));
			msg.value = (flag.ev_data_arrived ? (1 << 0) : 0)
				    | (flag.ekfs_converged ? (1 << 1) : 0)
				    | (flag.heading_ok ? (1 << 2) : 0)
				    | (flag.pos_ok ? (1 << 3) : 0)
				    | (static_cast<int32_t>(dstate) << 8);

			mavlink_msg_named_value_int_send_struct(_mavlink->get_channel(), &msg);

			// if (msg.value != _last_value) {
			// 	PX4_INFO("FEV_FLAG ch%d: ev=%d ekf=%d head=%d pos=%d dstate=%d",
			// 		 _mavlink->get_channel(), flag.ev_data_arrived, flag.ekfs_converged,
			// 		 flag.heading_ok, flag.pos_ok, flag.dstate);
			// 	_last_value = msg.value;
			// }

			return true;
		}

		return false;
	}
};

#endif // FORMIC_EV_FLAG_HPP
