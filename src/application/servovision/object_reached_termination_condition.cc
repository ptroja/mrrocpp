/*
 * object_reached_termination_condition.cc
 *
 *  Created on: Apr 29, 2010
 *      Author: mboryn
 */

#include "object_reached_termination_condition.h"
#include "lib/logger.h"

using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

object_reached_termination_condition::object_reached_termination_condition(double max_speed, double max_accel, int min_steps) :
	max_speed(max_speed), max_accel(max_accel), min_steps(min_steps), steps_below_max_speed(0)
{
}

object_reached_termination_condition::~object_reached_termination_condition()
{
}

bool object_reached_termination_condition::terminate_now()
{
	if(object_visible && current_speed.norm() <= max_speed && current_accel.norm() <= max_accel){
		steps_below_max_speed++;
	}
	else{
		steps_below_max_speed = 0;
	}

//	logDbg("bool object_reached_termination_condition::terminate_now() steps_below_max_speed: %5d\n", steps_below_max_speed);

	if(steps_below_max_speed >= min_steps){
		return true;
	}
	return false;
}

void object_reached_termination_condition::reset()
{
	steps_below_max_speed = 0;
}

}//namespace generator

}

}

}
