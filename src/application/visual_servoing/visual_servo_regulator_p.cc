/*
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#include "visual_servo_regulator_p.h"

namespace mrrocpp {
namespace ecp {
namespace servovision {

regulator_p::regulator_p(const lib::configurator & config, const std::string& config_section_name) :
	visual_servo_regulator(config, config_section_name)
{
	Kp = config.value <6, 6> ("regulator_kp_matrix", config_section_name);
	std::cout << "regulator_p: Kp:\n" << Kp << "\n\n";
	std::cout.flush();
}

regulator_p::~regulator_p()
{
}

const Eigen::Matrix <double, 6, 1> & regulator_p::compute_control(const Eigen::Matrix <double, 6, 1> & error, double dt)
{
	this->computed_control = Kp * error;
	return this->computed_control;
}

void regulator_p::reset()
{
}

} // namespace servovision
}
}
