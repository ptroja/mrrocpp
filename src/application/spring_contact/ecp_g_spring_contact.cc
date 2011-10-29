/*!
 * @file
 * @brief File contains ecp_generator class definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup spring_contact
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <cmath>

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_spring_contact.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			spring_contact_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

spring_contact::spring_contact(common::task::task& _ecp_task, int step) :
		common::generator::generator(_ecp_task), step_no(step), tool_frame(0.0, 0.0, 0.25)
{
}

bool spring_contact::first_step()
{

	std::cout << "spring_contact" << node_counter << std::endl;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	//	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::TOOL_FRAME;
	the_robot->ecp_command.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = td.internode_step_no;
	the_robot->ecp_command.value_in_step_no = td.value_in_step_no;

	the_robot->ecp_command.robot_model.tool_frame_def.tool_frame = tool_frame;

	for (int i = 0; i < 3; i++) {
		the_robot->ecp_command.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		the_robot->ecp_command.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;
		the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
		the_robot->ecp_command.arm.pf_def.reciprocal_damping[i + 3] = lib::TORQUE_RECIPROCAL_DAMPING;
	}

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = 0;
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		//	the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0.0;
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	the_robot->ecp_command.arm.pf_def.inertia[2] = lib::FORCE_INERTIA / 1;
	the_robot->ecp_command.arm.pf_def.reciprocal_damping[2] = lib::FORCE_RECIPROCAL_DAMPING / 1;
	the_robot->ecp_command.arm.pf_def.behaviour[2] = lib::CONTACT;
	// Sila dosciku do rawedzi
	the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[2] = 4;

	return true;
}
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
bool spring_contact::next_step()
{
	// static int count;
	// struct timespec start[9];
	if (check_and_null_trigger()) {
		return false;
	}

	std::cout << "spring_contact" << node_counter << std::endl;

	the_robot->ecp_command.instruction_type = lib::SET_GET;

	return true;

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
