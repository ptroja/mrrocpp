/*
 * visual_servo_regulator_pid.cc
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#include <iostream>

#include "visual_servo_regulator_pid.h"
#include "base/lib/logger.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

using namespace std;

regulator_pid::regulator_pid(const lib::configurator & config, const std::string& config_section_name) :
	visual_servo_regulator(config, config_section_name)
{
	error_t_1.setZero();
	error_t_2.setZero();
	Kp = config.value <6, 6> ("regulator_kp_matrix", config_section_name);
	Eigen::Matrix <double, 6, 6> Ti = config.value <6, 6> ("regulator_ti_matrix", config_section_name);
	Ti_1 = Ti.inverse();
	Td = config.value <6, 6> ("regulator_td_matrix", config_section_name);

	cout<< "\n====== Kp:\n" << Kp << endl;
	cout<< "\n====== Ti:\n" << Ti << endl;
	cout<< "\n====== Ti_1:\n" << Ti_1 << endl;
	cout<< "\n====== Td:\n" << Td << "\n\n";
}

regulator_pid::~regulator_pid()
{
}

const Eigen::Matrix <double, 6, 1> & regulator_pid::compute_control(const Eigen::Matrix <double, 6, 1> & error, double dt)
{
	Eigen::Matrix <double, 6, 1> A1, A2, A3;
	A1 = (Kp + Kp * dt * Ti_1 + Td / dt) * error;
	A2 = (-Kp - 2 * Kp * Td / dt) * error_t_1;
	A3 = Kp * Td / dt * error_t_2;
	computed_control = computed_control + A1 + A2 + A3;
//	computed_control = computed_control + Kp * ((1 + dt * Ti.inverse() + Td / dt) * error + (-1 - 2 * Td / dt) * error_t_1
//				+ Td / dt * error_t_2);

	error_t_2 = error_t_1;
	error_t_1 = error;

	return computed_control;
}

} //namespace
}
}
