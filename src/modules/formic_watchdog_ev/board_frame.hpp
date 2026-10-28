/****************************************************************************
 * The orientation of a pose from formic_odom in the body frame EKF2 works in.
 ****************************************************************************/

#pragma once

#include <matrix/math.hpp>

namespace formic_watchdog_ev
{

/**
 * Returns the rotation from the sensor frame to the body frame for an internal sensor.
 *
 * It is the rotation PX4 applies to the data of every internal IMU before EKF2 sees it
 * (calibration::Accelerometer::set_rotation): the level trim SENS_BOARD_X/Y/Z_OFF after the board
 * rotation SENS_BOARD_ROT.
 *
 *     body = level * board * sensor
 *
 * @param level the level trim as roll, pitch, yaw angles, from calibration::GetSensorLevelAdjustment()
 * @param board the board rotation, from calibration::GetBoardRotationMatrix()
 */
inline matrix::Dcmf body_from_sensor(const matrix::Eulerf &level, const matrix::Dcmf &board)
{
	return matrix::Dcmf(level) * board;
}

/**
 * Turns the orientation `q` of the sensor frame in NED into the orientation of the body frame in
 * NED, in place.
 *
 * The sender gives the orientation of the raw sensor frame: the frame of sensor_gyro and
 * sensor_accel, which the IMU calibration is made in. EKF2 works in the body frame.
 *
 *     R_ned_body = R_ned_sensor * transpose(body_from_sensor)
 *
 * @param q                 (w, x, y, z), the PX4 order
 * @param body_from_sensor  from body_from_sensor()
 * @return false and `q` unchanged when `q` is not finite or not close to a unit quaternion
 */
inline bool sensor_to_body_orientation(float q[4], const matrix::Dcmf &body_from_sensor)
{
	matrix::Quatf q_ned_sensor(q);

	if (!q_ned_sensor.isAllFinite() || q_ned_sensor.length() < 0.9f) {
		return false;
	}

	q_ned_sensor.normalize();
	const matrix::Dcmf R_ned_sensor(q_ned_sensor);
	const matrix::Dcmf R_ned_body(R_ned_sensor * body_from_sensor.T());
	matrix::Quatf q_ned_body(R_ned_body);
	q_ned_body.normalize();

	for (int i = 0; i < 4; i++) {
		q[i] = q_ned_body(i);
	}

	return true;
}

} // namespace formic_watchdog_ev
