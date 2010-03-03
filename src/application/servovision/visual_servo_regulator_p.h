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

class regulator_p: public mrrocpp::ecp::common::generator::visual_servo_regulator
{
public:
	regulator_p(const lib::configurator & config, const char * config_section_name, int error_size, int control_size);
	virtual ~regulator_p();
	const boost::numeric::ublas::vector<double> & calculate_control(const boost::numeric::ublas::vector<double> & error);
protected:
	boost::numeric::ublas::matrix<double> Kp;
}; // class regulator_p

} // namespace generator

}

}

}

#endif /* VISUAL_SERVO_REGULATOR_P_H_ */
