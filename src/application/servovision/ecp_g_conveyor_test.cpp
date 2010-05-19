/*
 * ecp_g_conveyor_test.cpp
 *
 *  Created on: May 19, 2010
 *      Author: mboryn
 */

#include "ecp_g_conveyor_test.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

ecp_g_conveyor_test::ecp_g_conveyor_test(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
	generator(ecp_task), motion_steps(30)
{
	dt = motion_steps * 0.002;
}

ecp_g_conveyor_test::~ecp_g_conveyor_test()
{

}

bool ecp_g_conveyor_test::first_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = motion_steps;
	the_robot->ecp_command.instruction.value_in_step_no = motion_steps - 3;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}
	return true;
}

bool ecp_g_conveyor_test::next_step()
{

	return true;
}

}//namespace

}

}

}
