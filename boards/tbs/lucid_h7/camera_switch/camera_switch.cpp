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

/**
 * @file camera_switch.cpp
 *
 * Selects the analog camera input (PD11) from an RC AUX channel.
 */

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <parameters/param.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/manual_control_setpoint.h>

#include "board_config.h"

using namespace time_literals;

class CameraSwitch : public ModuleBase<CameraSwitch>, public px4::ScheduledWorkItem
{
public:
	CameraSwitch() : ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default) {}
	~CameraSwitch() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);
	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

	static void select_camera(int camera);

private:
	void Run() override;

	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	int32_t _aux_channel{0};
	int _rc_camera{0}; // last camera selected by RC, 0 = none yet
};

void CameraSwitch::select_camera(int camera)
{
	// HIGH selects camera 1, LOW selects camera 2
	px4_arch_gpiowrite(GPIO_CAMSW, camera == 1);
}

void CameraSwitch::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	manual_control_setpoint_s manual_control_setpoint;

	if (_manual_control_setpoint_sub.update(&manual_control_setpoint) && manual_control_setpoint.valid) {
		const float aux[] = {
			manual_control_setpoint.aux1, manual_control_setpoint.aux2, manual_control_setpoint.aux3,
			manual_control_setpoint.aux4, manual_control_setpoint.aux5, manual_control_setpoint.aux6
		};

		const float value = aux[_aux_channel - 1];

		if (PX4_ISFINITE(value)) {
			const int camera = (value > 0.f) ? 1 : 2;

			// only write on change so a manual "set" holds until the switch moves
			if (camera != _rc_camera) {
				_rc_camera = camera;
				select_camera(camera);
			}
		}
	}

	// on RC loss nothing is written and the last camera stays selected
}

int CameraSwitch::task_spawn(int argc, char *argv[])
{
	int32_t aux_channel = 0;
	param_get(param_find("TBS_CAM_SWITCH"), &aux_channel);

	if (aux_channel < 1 || aux_channel > 6) {
		PX4_ERR("TBS_CAM_SWITCH must be 1-6 (is %" PRId32 ")", aux_channel);
		return PX4_ERROR;
	}

	CameraSwitch *instance = new CameraSwitch();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	instance->_aux_channel = aux_channel;
	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleOnInterval(50_ms); // 20 Hz

	return PX4_OK;
}

int CameraSwitch::print_status()
{
	PX4_INFO("AUX%" PRId32 ", RC selected camera: %d, PD11: %s", _aux_channel, _rc_camera,
		 px4_arch_gpioread(GPIO_CAMSW) ? "HIGH (camera 1)" : "LOW (camera 2)");
	return 0;
}

int CameraSwitch::custom_command(int argc, char *argv[])
{
	// "set" drives the pin directly, so it also works when the module isn't running
	if (argc >= 2 && strcmp(argv[0], "set") == 0) {
		const int camera = atoi(argv[1]);

		if (camera == 1 || camera == 2) {
			select_camera(camera);
			return PX4_OK;
		}
	}

	return print_usage("unknown command");
}

int CameraSwitch::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Switches the TBS Lucid H7 analog camera input (PD11) from an RC AUX channel.
AUX above 0 selects camera 1, otherwise camera 2. The AUX channel is set with TBS_CAM_SWITCH.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("camera_switch", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Select a camera manually");
	PRINT_MODULE_USAGE_ARG("1|2", "Camera", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int camera_switch_main(int argc, char *argv[])
{
	return CameraSwitch::main(argc, argv);
}
