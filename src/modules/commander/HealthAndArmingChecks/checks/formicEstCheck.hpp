#pragma once

#include "../Common.hpp"

/**
 * Checks that the parameters match the Formic estimator design.
 *
 * EKF2 runs as instance 0 behind the selector and formic_vio_est as instance 1, so EKF2 must
 * run in multi mode with exactly one instance.
 */
class FormicEstChecks : public HealthAndArmingCheckBase
{
public:
	FormicEstChecks();
	~FormicEstChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

	void updateParams() override;

private:
	param_t _param_sens_imu_mode{PARAM_INVALID};
	param_t _param_ekf2_multi_imu{PARAM_INVALID};
	param_t _param_ekf2_multi_mag{PARAM_INVALID};
	param_t _param_formic_vio_src{PARAM_INVALID};
	param_t _param_ekf2_ev_ctrl{PARAM_INVALID};

	int32_t _sens_imu_mode{1};
	int32_t _ekf2_multi_imu{0};
	int32_t _ekf2_multi_mag{0};
	int32_t _formic_vio_src{0};
	int32_t _ekf2_ev_ctrl{0};
};
