/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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
#pragma once

#include <drivers/drv_dshot.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include "DShotTelemetry.h"

// Turtle (crash-flip) recovery: disarmed-only motor reversal via DShot 20/21
#include <lib/turtle_recovery/TurtleCore.hpp>
#include <lib/turtle_recovery/TurtleMapping.hpp>
#include <lib/turtle_recovery/TurtleHelpers.hpp>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

#if !defined(DIRECT_PWM_OUTPUT_CHANNELS)
#  error "board_config.h needs to define DIRECT_PWM_OUTPUT_CHANNELS"
#endif

/** Dshot PWM frequency, Hz */
static constexpr unsigned int DSHOT150  =  150000u;
static constexpr unsigned int DSHOT300  =  300000u;
static constexpr unsigned int DSHOT600  =  600000u;

static constexpr int DSHOT_DISARM_VALUE = 0;
static constexpr int DSHOT_MIN_THROTTLE = 1;
static constexpr int DSHOT_MAX_THROTTLE = 1999;

class DShot final : public ModuleBase<DShot>, public OutputModuleInterface
{
public:
	DShot();
	~DShot() override;

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	int init();

	void mixerChanged() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * Send a dshot command to one or all motors
	 * This is expected to be called from another thread.
	 * @param num_repetitions number of times to repeat, set at least to 1
	 * @param motor_index index or -1 for all
	 * @return 0 on success, <0 error otherwise
	 */
	int send_command_thread_safe(const dshot_command_t command, const int num_repetitions, const int motor_index);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	bool telemetry_enabled() const { return _telemetry != nullptr; }

	bool updateOutputs(uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:

	/** Disallow copy construction and move assignment. */
	DShot(const DShot &) = delete;
	DShot operator=(const DShot &) = delete;

	enum class DShotConfig {
		Disabled  = 0,
		DShot150  = 150,
		DShot300  = 300,
		DShot600  = 600,
	};

	struct Command {
		dshot_command_t command{};
		int num_repetitions{0};
		uint8_t motor_mask{0xff};
		bool save{false};

		bool valid() const { return num_repetitions > 0; }
		void clear() { num_repetitions = 0; }
	};

	int _last_telemetry_index{-1};
	uint8_t _actuator_functions[esc_status_s::CONNECTED_ESC_MAX] {};

	void enable_dshot_outputs(const bool enabled);

	void init_telemetry(const char *device, bool swap_rxtx);

	int handle_new_telemetry_data(const int telemetry_index, const DShotTelemetry::EscData &data, bool ignore_rpm);

	void publish_esc_status(void);

	int handle_new_bdshot_erpm(void);

	void Run() override;

	void update_params();

	void update_num_motors();

	void handle_vehicle_commands();

	uint16_t convert_output_to_3d_scaling(uint16_t output);

	/**
	 * Turtle recovery: refresh the TRTL_* parameter snapshot (4 Hz, only while
	 * idle, disarmed and switch OFF) and the gated inputs for the controller.
	 */
	void update_turtle_parameters();
	void update_turtle_inputs();

	MixingOutput _mixing_output{PARAM_PREFIX, DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};
	uint32_t _reversible_outputs{};

	// Turtle recovery controller state (see lib/turtle_recovery)
	turtle::Core _trtl_core{turtle::interlock()};
	turtle::Config _trtl_config{};
	turtle::Input _trtl_input{};
	turtle::Frame _trtl_frame{};
	turtle_mapping::Config _trtl_map_config{};
	turtle_mapping::Resolved _trtl_map{};
	bool _trtl_loaded{false};
	bool _trtl_map_ok{false};
	bool _trtl_geometry_ok{false};
	bool _trtl_rc_conflict{true};
	bool _trtl_output_seen{false};
	bool _trtl_fallback{false};
	bool _trtl_requested{false};
	uint64_t _trtl_last_params{0};
	static constexpr unsigned TRTL_PARAM_COUNT = 22;
	param_t _trtl_handles[TRTL_PARAM_COUNT] {};
	bool _trtl_handles_initialized{false};
	param_t _trtl_enable_handle{PARAM_INVALID};

	uORB::Subscription _trtl_manual_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _trtl_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _trtl_armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _trtl_att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _trtl_rc_sub{ORB_ID(rc_channels)};
	manual_control_setpoint_s _trtl_manual{};
	vehicle_status_s _trtl_status{};
	actuator_armed_s _trtl_armed{};
	vehicle_attitude_s _trtl_att{};
	rc_channels_s _trtl_rc_channels{};

	DShotTelemetry *_telemetry{nullptr};

	uORB::PublicationMultiData<esc_status_s> esc_status_pub{ORB_ID(esc_status)};

	static char _telemetry_device[20];
	static bool _telemetry_swap_rxtx;
	static px4::atomic_bool _request_telemetry_init;

	px4::atomic<Command *> _new_command{nullptr};


	bool _outputs_initialized{false};
	bool _outputs_on{false};
	bool _bidirectional_dshot_enabled{false};

	static constexpr unsigned _num_outputs{DIRECT_PWM_OUTPUT_CHANNELS};
	uint32_t _output_mask{0};

	int _num_motors{0};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_bdshot_rpm_perf{perf_alloc(PC_COUNT, MODULE_NAME": bdshot rpm")};
	perf_counter_t	_dshot_telem_perf{perf_alloc(PC_COUNT, MODULE_NAME": dshot telem")};

	Command _current_command{};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};
	uint16_t _esc_status_counter{0};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DSHOT_MIN>)    _param_dshot_min,
		(ParamBool<px4::params::DSHOT_3D_ENABLE>) _param_dshot_3d_enable,
		(ParamInt<px4::params::DSHOT_3D_DEAD_H>) _param_dshot_3d_dead_h,
		(ParamInt<px4::params::DSHOT_3D_DEAD_L>) _param_dshot_3d_dead_l,
		(ParamInt<px4::params::MOT_POLE_COUNT>) _param_mot_pole_count,
		(ParamBool<px4::params::DSHOT_BIDIR_EN>) _param_bidirectional_enable
	)
};
