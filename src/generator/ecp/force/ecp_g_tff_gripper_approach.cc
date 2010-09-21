/*!
 * @file
 * @brief File contains tff_gripper_approach generator definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup generators
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <cmath>
#include <iostream>

#include "base/lib/typedefs.h"

#include "base/lib/sr/srlib.h"
#include "base/ecp/ecp_robot.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

tff_gripper_approach::tff_gripper_approach(common::task::task& _ecp_task, int step) :
	generator(_ecp_task), step_no(step)
{
}

void tff_gripper_approach::configure(double l_speed, unsigned int l_motion_time)
{
	speed = l_speed;
	motion_time = l_motion_time;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_gripper_approach::first_step()
{
	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame);

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	//	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::RELATIVE;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
	}

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = 0;
	}

	the_robot->ecp_command.instruction.arm.pf_def.inertia[2] = lib::FORCE_INERTIA / 4;
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[2] = speed;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		//		the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0;
	}

	the_robot->ecp_command.instruction.arm.pf_def.behaviour[2] = lib::GUARDED_MOTION;
	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[2] = lib::FORCE_RECIPROCAL_DAMPING / 2;

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_gripper_approach::next_step()
{
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter == 1) {

	} else if (node_counter > motion_time) {
		return false;
	}

	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
