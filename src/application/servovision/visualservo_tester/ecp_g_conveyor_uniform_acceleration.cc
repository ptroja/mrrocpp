/*
 * ecp_g_conveyor_sinus.cpp
 *
 *  Created on: May 20, 2010
 *      Author: mboryn
 */

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_conveyor_uniform_acceleration.h"
#include <cmath>
#include <stdexcept>

#include "base/lib/logger.h"

using namespace std;
using namespace logger;

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

ecp_g_conveyor_uniform_acceleration::ecp_g_conveyor_uniform_acceleration(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
	generator(ecp_task)
{
	motion_steps = 30;
	dt = motion_steps * 0.002;
	acceleration = ecp_task.config.value <double> ("acceleration", section_name);
	max_speed = ecp_task.config.value <double> ("max_speed", section_name);

	if (!(max_speed * acceleration > 0)) {
		throw runtime_error("ecp_g_conveyor_uniform_acceleration: !(max_speed * acceleration > 0)");
	}

	t = 0;
	current_speed = 0;
}

ecp_g_conveyor_uniform_acceleration::~ecp_g_conveyor_uniform_acceleration()
{
}

bool ecp_g_conveyor_uniform_acceleration::first_step()
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
	t = 0;
	current_speed = 0;

	log_dbg("bool ecp_g_conveyor_uniform_acceleration::first_step()\n");

	return true;
}
bool ecp_g_conveyor_uniform_acceleration::next_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	if (!initial_position_saved) {
		current_position = the_robot->reply_package.arm.pf_def.arm_coordinates[0];
		initial_position_saved = true;
	}

	double dv = acceleration * dt;

	if (fabs(current_speed) < max_speed) {
		current_speed = current_speed + dv;
	} else {
		current_speed = max_speed;
	}

	double ds = current_speed * dt;

	current_position = current_position + ds;

	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = current_position;

	t += dt;
	return true;
}

}//namespace

}

}

}
