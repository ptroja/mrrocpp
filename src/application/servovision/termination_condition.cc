/*
 * termination_condition.cc
 *
 *  Created on: Apr 29, 2010
 *      Author: mboryn
 */

#include "termination_condition.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

termination_condition::termination_condition() :
	object_visible(false)
{
	current_accel.setZero();
	current_speed.setZero();
}

termination_condition::~termination_condition()
{
}

void termination_condition::update_end_effector_speed(const Eigen::Matrix<double, 3, 1>&  current_speed)
{
	this->current_speed = current_speed;
}
void termination_condition::update_end_effector_accel(const Eigen::Matrix<double, 3, 1>&  current_accel)
{
	this->current_accel = current_accel;
}
void termination_condition::update_object_visibility(bool object_visible)
{
	this->object_visible = object_visible;
}

}//namespace generator

}

}

}
