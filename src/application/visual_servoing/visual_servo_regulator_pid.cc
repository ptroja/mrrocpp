/*
 * visual_servo_regulator_pid.cc
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#include <iostream>
#include <stdexcept>
#include <algorithm>

#include "visual_servo_regulator_pid.h"
#include "base/lib/logger.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

using namespace std;

regulator_pid::regulator_pid(const lib::configurator & config, const std::string& config_section_name) :
	visual_servo_regulator(config, config_section_name)
{
	cout<<"================= regulator_pid::regulator_pid() begin\n\n";
	error_t_1.setZero();
	error_integral.setZero();

	Kp = config.value <6, 6> ("regulator_kp_matrix", config_section_name);
	Ki = config.value <6, 6> ("regulator_ki_matrix", config_section_name);
	Kd = config.value <6, 6> ("regulator_kd_matrix", config_section_name);

	max_error_integral = config.value <6, 1> ("max_error_integral", config_section_name);
	min_error_integral = -max_error_integral;

	for (int i = 0; i < max_error_integral.rows(); ++i) {
		if (max_error_integral(i, 0) < 0) {
			throw runtime_error("regulator_pid: max_error_integral(i, 0) < 0");
		}
	}

	cout << "\n====== Kp:\n" << Kp << endl;
	cout << "\n====== Ki:\n" << Ki << endl;
	cout << "\n====== Kd:\n" << Kd << "\n\n";

	cout << "\n====== max_error_integral:\n" << max_error_integral << "\n";
	cout << "\n====== min_error_integral:\n" << min_error_integral << "\n\n";
}

regulator_pid::~regulator_pid()
{
}

const Eigen::Matrix <double, 6, 1> & regulator_pid::compute_control(const Eigen::Matrix <double, 6, 1> & error, double dt)
{
	error_integral += error * dt;
	for (int i = 0; i < error_integral.rows(); ++i) { // integral constraints
		error_integral(i, 0) = min(error_integral(i, 0), max_error_integral(i, 0));
		error_integral(i, 0) = max(error_integral(i, 0), min_error_integral(i, 0));
	}

	Eigen::Matrix <double, 6, 1> error_derivative = (error - error_t_1) / dt;

	computed_control = Kp * error + Ki * error_integral + Kd * error_derivative;

	error_t_1 = error;

	return computed_control;
}

void regulator_pid::reset()
{
	error_t_1.setZero();
	error_integral.setZero();
}

} //namespace
}
}
