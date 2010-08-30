/*
 * object_reached_termination_condition.h
 *
 *  Created on: Apr 29, 2010
 *      Author: mboryn
 */

#ifndef OBJECT_REACHED_TERMINATION_CONDITION_H_
#define OBJECT_REACHED_TERMINATION_CONDITION_H_

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
	object_reached_termination_condition(double max_speed, double max_accel, int min_steps);
	virtual ~object_reached_termination_condition();
	virtual void reset();
	virtual void update(const mrrocpp::ecp::common::generator::visual_servo_manager* vsm);
	virtual bool is_condition_met() const;
protected:
	double max_speed;
	double max_accel;
	int min_steps;
	int steps_below_max_speed;
};

/** @} */

}//namespace generator

}

}

#endif /* OBJECT_REACHED_TERMINATION_CONDITION_H_ */
