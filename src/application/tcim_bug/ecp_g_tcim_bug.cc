/*
 * ecp_g_tcim_bug.cc
 *
 *  Created on: 07-06-2011
 *      Author: mboryn
 */

#include "ecp_g_tcim_bug.h"

namespace mrrocpp {

namespace ecp {

namespace common {

namespace generator {

ecp_g_tcim_bug::ecp_g_tcim_bug(mrrocpp::ecp::common::task::task & ecp_task) :
	mrrocpp::ecp::common::generator::generator(ecp_task)
{
}

ecp_g_tcim_bug::~ecp_g_tcim_bug()
{
}

bool ecp_g_tcim_bug::first_step()
{
	motion_steps = 20;

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.set_arm_type = lib::FRAME;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = motion_steps;
	the_robot->ecp_command.value_in_step_no = motion_steps - 3;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	current_position_saved = false;

	clock_gettime(CLOCK_REALTIME, &prev_timestamp);
	c = 0;
	ss.str("");
	ss << "first_step(): motion_steps = " << the_robot->ecp_command.motion_steps << "    value_in_step_no = "
			<< the_robot->ecp_command.value_in_step_no << "\n";

	return true;
}

bool ecp_g_tcim_bug::next_step()
{
	clock_gettime(CLOCK_REALTIME, &current_timestamp);
	int sec = current_timestamp.tv_sec - prev_timestamp.tv_sec;
	int nsec = current_timestamp.tv_nsec - prev_timestamp.tv_nsec;

	double next_step_time = sec + 1e-9 * nsec;

	if (!current_position_saved) { // save first position
		current_position = the_robot->reply_package.arm.pf_def.arm_frame;
		current_position_saved = true;
	}

	if (c > 10 && c % 4 == 0) {
		motion_steps = 21;
	} else {
		motion_steps = 20;
	}

	the_robot->ecp_command.instruction_type = lib::SET_GET;
	the_robot->ecp_command.arm.pf_def.arm_frame = current_position;
	the_robot->ecp_command.motion_steps = motion_steps;
	the_robot->ecp_command.value_in_step_no = motion_steps - 3;

	ss << "\n";
	ss << "next_step_time = " << next_step_time << "\n";
	if (next_step_time < 0.005) {
		ss << "----------------------------------------- next_step_time < 0.005\n";
	}
	ss << "motion_steps = " << the_robot->ecp_command.motion_steps << "    value_in_step_no = "
			<< the_robot->ecp_command.value_in_step_no << "\n";

	if (++c > 200) {
		c = 0;
		std::cout << "\n\n============================\n" << ss.str() << "============================\n\n";
		ss.str("");
		return false;
	}

	prev_timestamp = current_timestamp;

	return true;
}

}

}

}

}
