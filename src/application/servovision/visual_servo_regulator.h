/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_REGULATOR_H_
#define VISUAL_SERVO_REGULATOR_H_

#include <string>

#include <Eigen/Core>

#include "lib/configurator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

/**
 * Abstract class for regulators used in servovision. Regulator calculates control supplied to servovision generator
 */
template <int ERROR_SIZE, int CONTROL_SIZE>
class visual_servo_regulator
{
public:
	virtual ~visual_servo_regulator()
	{
	}
	;

	virtual const Eigen::Matrix <double, CONTROL_SIZE, 1> & calculate_control(const Eigen::Matrix <double, ERROR_SIZE,
			1> & error) = 0;

	const Eigen::Matrix <double, CONTROL_SIZE, 1> & get_control()
	{
		return calculated_control;
	}

protected:
	visual_servo_regulator(const lib::configurator & config, const char * config_section_name) :
		config(config), config_section_name(config_section_name)
	{
	}

	const lib::configurator & config;

	const std::string config_section_name;

	Eigen::Matrix <double, CONTROL_SIZE, 1> calculated_control;
private:

}; // class visual_servo_regulator

} // namespace generator

} // namespace common

} // namespace ecp

} // namespace mrrocpp

#endif /* VISUAL_SERVO_REGULATOR_H_ */
