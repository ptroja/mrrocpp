/*
 * $Id$
 *
 *  Created on: Mar 3, 2010
 *      Author: mboryn
 */

#ifndef VISUAL_SERVO_REGULATOR_P_H_
#define VISUAL_SERVO_REGULATOR_P_H_

#include "visual_servo_regulator.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

template <int ERROR_SIZE, int CONTROL_SIZE>
class regulator_p: public mrrocpp::ecp::common::generator::visual_servo_regulator <ERROR_SIZE, CONTROL_SIZE>
{
public:
	regulator_p(const lib::configurator & config, const char * config_section_name) :
		visual_servo_regulator<CONTROL_SIZE, ERROR_SIZE>(config, config_section_name)
	{
		Kp = config.value <CONTROL_SIZE, ERROR_SIZE> ("regulator_kp_matrix", config_section_name);
		std::cout << "regulator_p: Kp:\n" << Kp << "\n\n"; std::cout.flush();

	}

	virtual ~regulator_p()
	{
	}
	virtual const Eigen::Matrix <double, CONTROL_SIZE, 1> & calculate_control(const Eigen::Matrix <double, ERROR_SIZE, 1> & error)
	{
		//std::cout << "regulator_p::calculate_control() begin\n"; std::cout.flush();
		this->calculated_control = Kp * error;
		//std::cout << "regulator_p::calculate_control() end\n"; std::cout.flush();
		return this->calculated_control;
	}
protected:
	Eigen::Matrix <double, CONTROL_SIZE, ERROR_SIZE> Kp;
}; // class regulator_p

} // namespace generator

}

}

}

#endif /* VISUAL_SERVO_REGULATOR_P_H_ */
