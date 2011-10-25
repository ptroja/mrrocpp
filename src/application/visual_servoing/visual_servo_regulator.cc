/*
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "visual_servo_regulator.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

visual_servo_regulator::visual_servo_regulator(const lib::configurator & config, const std::string& config_section_name) :
	config(config), config_section_name(config_section_name)
{
	computed_control.setZero();
}

visual_servo_regulator::~visual_servo_regulator()
{
}

const Eigen::Matrix <double, 6, 1> & visual_servo_regulator::get_control()
{
	return computed_control;
}

} // namespace servovision
} // namespace ecp
} // namespace mrrocpp
