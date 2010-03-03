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

regulator_p::regulator_p(const lib::configurator & config, const char * config_section_name, int error_size, int control_size) :
	visual_servo_regulator(config, config_section_name, error_size, control_size), Kp(control_size, error_size)
{
	Kp = config.value("regulator_kp_matrix", config_section_name, control_size, error_size);
	std::cout << "regulator_p: Kp: " << Kp << std::endl;
}

regulator_p::~regulator_p()
{
	// TODO Auto-generated destructor stub
}

const boost::numeric::ublas::vector<double> & regulator_p::calculate_control(const boost::numeric::ublas::vector<double> & error)
{
	calculated_control = prod(Kp, error);
	return calculated_control;
}

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp
