/*
 * visual_servo_regulator_pid.cc
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#include "visual_servo_regulator_pid.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

regulator_pid::regulator_pid(const lib::configurator & config, const std::string& config_section_name) :
	visual_servo_regulator(config, config_section_name)
{
	error_t_1.setZero();
	error_t_2.setZero();
	Kp = config.value <6, 6> ("regulator_kp_matrix", config_section_name);
	Ki = config.value <6, 6> ("regulator_ki_matrix", config_section_name);
	Kd = config.value <6, 6> ("regulator_kd_matrix", config_section_name);
}

regulator_pid::~regulator_pid()
{
}

const Eigen::Matrix <double, 6, 1> & regulator_pid::compute_control(const Eigen::Matrix <double, 6, 1> & error, double dt)
{
	//	calculated_control = calculated_control + Kp * ((1 + dt / Ti + Td / dt) * error + (-1 - 2 * Td / dt) * error_t_1
	//			+ Td / dt * error_t_2);

	computed_control = computed_control + (Kp + Ki * dt + Kd / dt) * error + (-Kp - Kd * (2 / dt)) * error_t_1 + Kd
			* error_t_2 / dt;

	error_t_2 = error_t_1;
	error_t_1 = error;

	return computed_control;
}

} //namespace
}
}
