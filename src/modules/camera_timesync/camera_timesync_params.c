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
 * Enable the strobe-based camera timesync.
 *
 * Matches companion camera frame stamps (camera_frame_stamp) with the
 * camera_capture strobe edges (camera_trigger) and estimates the clock
 * offset between the companion and the flight controller.
 * Needs CAM_CAP_FBACK=1 and a PWM channel set to Camera_Capture.
 *
 * @boolean
 * @reboot_required true
 * @group Camera Timesync
 */
PARAM_DEFINE_INT32(CAMTS_EN, 0);

/**
 * Apply the camera timesync offset to the uXRCE-DDS client.
 *
 * 0: monitor only. The DDS client keeps its own round-trip estimate and
 * camera_timesync_status only reports the comparison.
 * 1: while locked, the DDS client converts every timestamp with the
 * strobe-based offset instead of the round-trip estimate.
 *
 * @boolean
 * @group Camera Timesync
 */
PARAM_DEFINE_INT32(CAMTS_APPLY, 1);

/**
 * Half-width of the strobe match window.
 *
 * A frame stamp is matched to the strobe closest to its predicted
 * flight-controller time. The match is accepted when the distance is
 * below this value. 0 selects 45% of the measured strobe period, which
 * makes the match unambiguous at any frame rate.
 *
 * @unit us
 * @min 0
 * @max 100000
 * @group Camera Timesync
 */
PARAM_DEFINE_INT32(CAMTS_WIN_US, 0);

/**
 * Maximum deviation of one measurement from the estimate.
 *
 * While locked, a matched sample whose offset differs from the current
 * estimate by more than this is rejected. More than 20 rejections in a
 * row drop the lock and restart the acquisition.
 *
 * @unit us
 * @min 100
 * @max 100000
 * @group Camera Timesync
 */
PARAM_DEFINE_INT32(CAMTS_MAX_DEV, 2000);

/**
 * Filter gain of the offset estimate.
 *
 * Exponential smoothing: estimate += gain * (observed - estimate),
 * once per matched frame. 1.0 follows every measurement; small values
 * smooth more and follow clock drift more slowly.
 *
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @group Camera Timesync
 */
PARAM_DEFINE_FLOAT(CAMTS_ALPHA, 0.1f);

/**
 * Consecutive matched frames needed to lock.
 *
 * The estimate starts from the median of these samples.
 *
 * @min 2
 * @max 60
 * @group Camera Timesync
 */
PARAM_DEFINE_INT32(CAMTS_LOCK_N, 10);

/**
 * Time without an accepted sample before the lock is dropped.
 *
 * After it the module reports STATE_STALE and apply=false, so the DDS
 * client falls back to its round-trip estimate.
 *
 * @unit ms
 * @min 100
 * @max 60000
 * @group Camera Timesync
 */
PARAM_DEFINE_INT32(CAMTS_TIMEOUT, 2000);

/**
 * Fixed delay added to the measured offset.
 *
 * Compensates a constant delay between the strobe edge the flight
 * controller captures and the start-of-frame time the companion driver
 * stamps. Positive moves the companion stamps later. Leave 0 when the
 * companion stamps every vision message with the same V4L2 frame time,
 * because then the delay cancels out.
 *
 * @unit us
 * @min -100000
 * @max 100000
 * @group Camera Timesync
 */
PARAM_DEFINE_INT32(CAMTS_DELAY_US, 0);
