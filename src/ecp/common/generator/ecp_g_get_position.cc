/*
 * ecp_g_get_position.cpp
 *
 *  Created on: Apr 29, 2010
 *      Author: rtulwin
 */

#include "ecp_g_get_position.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

get_position::get_position(common::task::task& _ecp_task, bool _is_synchronised, lib::POSE_SPECIFICATION pose_spec, int axes_num) :
        delta (_ecp_task)
{
	position = new double[axes_num];
	this->pose_spec = pose_spec;
}

get_position::~get_position()
{
	delete position;
}

bool get_position::first_step() {
	the_robot->ecp_command.instruction.get_arm_type = pose_spec;
}

bool get_position::next_step() {

}

double * get_position::get_position_array() {
	return position;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
