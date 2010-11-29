/*
 * object_reached_termination_condition.h
 *
 *  Created on: Apr 29, 2010
 *      Author: mboryn
 */

#ifndef OBJECT_REACHED_TERMINATION_CONDITION_H_
#define OBJECT_REACHED_TERMINATION_CONDITION_H_

#include <string>
#include "base/lib/configurator.h"

#include "termination_condition.h"

namespace mrrocpp {

namespace ecp {

namespace servovision {

/** @addtogroup servovision
 *  @{
 */

/**
 *
 */
class object_reached_termination_condition: public termination_condition
{
public:
	object_reached_termination_condition(const lib::configurator& config, const std::string &section_name);
	virtual ~object_reached_termination_condition();
	virtual void reset();
	virtual void update(const mrrocpp::ecp::common::generator::visual_servo_manager* vsm);
protected:
	double max_linear_speed;
	double max_angular_speed;
	double max_linear_accel;
	double max_angular_accel;
	double max_linear_error;
	double max_angular_error;
	int min_steps;
	int steps_delay;
};

/** @} */

}//namespace generator

}

}

#endif /* OBJECT_REACHED_TERMINATION_CONDITION_H_ */
