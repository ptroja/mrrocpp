/*
 * visual_servo_regulator_pid.cc
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#include "visual_servo_regulator_pid.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

regulator_pid::regulator_pid(const lib::configurator & config, const std::string& config_section_name) :
	visual_servo_regulator(config, config_section_name)
{

}

regulator_pid::~regulator_pid()
{
}

virtual const Eigen::Matrix <double, 6, 1> & calculate_control(const Eigen::Matrix <double, 6, 1> & error, double dt)
{

}

} //namespace

}

}

}
