/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include <iostream>

#include "visual_servo_regulator_p.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

//template <int ERROR_SIZE, int CONTROL_SIZE>
//regulator_p<ERROR_SIZE, CONTROL_SIZE>::regulator_p(const lib::configurator & config, const char * config_section_name, int error_size, int control_size) :
//	visual_servo_regulator(config, config_section_name, error_size, control_size), Kp(control_size, error_size)
//{
//	Kp = config.value<CONTROL_SIZE, ERROR_SIZE>("regulator_kp_matrix", config_section_name, control_size, error_size);
//	std::cout << "regulator_p: Kp: " << Kp << std::endl;
//}
//
//template <int ERROR_SIZE, int CONTROL_SIZE>
//regulator_p<ERROR_SIZE, CONTROL_SIZE>::~regulator_p()
//{
//	// TODO Auto-generated destructor stub
//}
//
//template <int ERROR_SIZE, int CONTROL_SIZE>
//const Eigen::Matrix<double, CONTROL_SIZE, 1> & regulator_p<ERROR_SIZE, CONTROL_SIZE>::calculate_control(const boost::numeric::ublas::vector<double> & error)
//{
//	calculated_control = Kp * error;
//	return calculated_control;
//}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp
