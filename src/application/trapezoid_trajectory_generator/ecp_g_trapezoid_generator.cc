/*
 * ecp_g_trapezoid_generator.cpp
 *
 *  Created on: 13-01-2011
 *      Author: mboryn
 */

#include "ecp_g_trapezoid_generator.h"
#include "base/ecp/ecp_robot.h"

namespace mrrocpp {
namespace ecp {
namespace trapezoid {

using namespace logger;

trapezoid_generator::trapezoid_generator(mrrocpp::ecp::common::task::task & ecp_task) :
	generator(ecp_task), motion_steps(30)
{
	//axis_idx = ecp_task.config.
}

trapezoid_generator::~trapezoid_generator()
{
}

bool trapezoid_generator::first_step()
{
	log_dbg("trapezoid_generator::first_step() begin\n");

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = motion_steps;
	the_robot->ecp_command.instruction.value_in_step_no = motion_steps - 3;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	current_theta_saved = false;

	log_dbg("trapezoid_generator::first_step() end\n");

	return true;
}

bool trapezoid_generator::next_step()
{
	if (!current_theta_saved) { // save first position
		for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i) {
			current_arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
		}
		current_theta_saved = true;
	}

	current_arm_coordinates[axis_idx] += 0.01;

	// prepare command to EDP
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
	for (size_t i = 0; i < lib::MAX_SERVOS_NR; ++i) {
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = current_arm_coordinates[i];
	}
	return true;
}

void trapezoid_generator::set_params(int axis_idx, double accel1, double accel2, double v_max, double time_v_max)
{
	this->axis_idx = axis_idx;
	this->accel1 = accel1;
	this->accel2 = accel2;
	this->v_max = v_max;
	this->time_v_max = time_v_max;
}

} // namespace trapezoid

}

}
