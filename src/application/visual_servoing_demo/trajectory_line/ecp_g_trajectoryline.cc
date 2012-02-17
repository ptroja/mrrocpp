/*
 * ecp_g_trajectoryline.cc
 *
 *  Created on: 11-08-2011
 *      Author: mateusz
 */

#include "ecp_g_trajectoryline.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

ecp_g_trajectory_line::ecp_g_trajectory_line(mrrocpp::ecp::common::task::task & ecp_task, const std::string& section_name) :
		common::generator::generator(ecp_task), initial_position_saved(false)
{
	motion_steps = ecp_task.config.value <int>("motion_steps", section_name);
	motion_steps_value_in_step_no = motion_steps - 3;
	dt = motion_steps * 2e-3;

	A = ecp_task.config.value <double>("A", section_name);
	f = ecp_task.config.value <double>("f", section_name);

}

ecp_g_trajectory_line::~ecp_g_trajectory_line()
{
}

bool ecp_g_trajectory_line::first_step()
{
	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = motion_steps;
	the_robot->ecp_command.value_in_step_no = motion_steps - motion_steps_value_in_step_no;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	initial_position_saved = false;

	return true;
}

bool ecp_g_trajectory_line::next_step()
{
	if (!initial_position_saved) { // save first position
		initial_position = the_robot->reply_package.arm.pf_def.arm_frame;
		initial_position_saved = true;
	}

	lib::Homog_matrix next_position = initial_position;

	initial_position(1, 3) += 5e-3;

	the_robot->ecp_command.instruction_type = lib::SET_GET;
	the_robot->ecp_command.arm.pf_def.arm_frame = next_position;
	t += dt;
	return true;
}

} /* namespace generator */
} /* namespace common */
} /* namespace ecp */
} /* namespace mrrocpp */
