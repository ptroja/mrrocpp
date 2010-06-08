/*
 * ecp_g_conveyor_sinus.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "ecp_g_conveyor_sinus.h"
#include <math.h>

#include "lib/logger.h"

using namespace std;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

ecp_g_conveyor_sinus::ecp_g_conveyor_sinus(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
	generator(ecp_task)
{
	motion_steps = 30;
	dt = motion_steps * 0.002;
	A = ecp_task.config.value <double> ("sinus_A", section_name);
	f = ecp_task.config.value <double> ("sinus_f", section_name);
	t = 0;
	initial_position = 0;
}

ecp_g_conveyor_sinus::~ecp_g_conveyor_sinus()
{
}

bool ecp_g_conveyor_sinus::first_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = motion_steps;
	the_robot->ecp_command.instruction.value_in_step_no = motion_steps - 3;
	//the_robot->ecp_command.instruction.set_robot_model_type = lib::ARM_KINEMATIC_MODEL;

	//	for (int i = 0; i < 6; i++) {
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::UNGUARDED_MOTION;
	//	}

	initial_position_saved = false;

	log_dbg("bool ecp_g_conveyor_sinus::first_step()\n");

	return true;
}
bool ecp_g_conveyor_sinus::next_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	if(!initial_position_saved){
		initial_position = the_robot->reply_package.arm.pf_def.arm_coordinates[0];
		initial_position_saved = true;
	}

	double new_position = A * (1 - cos(2 * M_PI * f * t));

//	log_dbg("bool ecp_g_conveyor_sinus::next_step(): new_position = %+8.6lg     initial_position = %+8.6lg\n", new_position, initial_position);

	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = initial_position + new_position;

	t += dt;
	return true;
}

}//namespace

}

}

}
