/*
 * visual_servo_regulator_pid.h
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_REGULATOR_PID_H_
#define VISUAL_SERVO_REGULATOR_PID_H_

#include "visual_servo_regulator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

class regulator_pid : public mrrocpp::ecp::common::generator::visual_servo_regulator
{
public:
	regulator_pid(const lib::configurator & config, const std::string& config_section_name);
	virtual ~regulator_pid();

	virtual const Eigen::Matrix <double, 6, 1> & calculate_control(const Eigen::Matrix <double, 6, 1> & error, double dt);
};

}

}

}

}

#endif /* VISUAL_SERVO_REGULATOR_PID_H_ */
