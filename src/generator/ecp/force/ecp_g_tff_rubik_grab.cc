/*!
 * @file
 * @brief File contains tff_rubik_grab generator definition
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
#include "generator/ecp/force/ecp_g_tff_rubik_grab.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

tff_rubik_grab::tff_rubik_grab(common::task::task& _ecp_task, int step) :
	generator(_ecp_task), step_no(step)
{
}

void tff_rubik_grab::configure(double l_goal_position, double l_position_increment, unsigned int l_min_node_counter, bool l_both_axes_running)
{
	goal_position = l_goal_position;
	position_increment = l_position_increment;
	min_node_counter = l_min_node_counter;
	both_axes_running = l_both_axes_running;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_grab::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

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

	for (int i = 0; i < 3; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;
	}

	if (both_axes_running)
		for (int i = 0; i < 2; i++) {
			the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
			the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::CONTACT;
		}
	else {
		the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[1] = lib::FORCE_RECIPROCAL_DAMPING;
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[1] = lib::CONTACT;
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::UNGUARDED_MOTION;
	}

	for (int i = 2; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_grab::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	//	  cout << "next_step" << endl;


	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	/*
	 if (node_counter == 1) {

	 the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = 0;
	 desired_absolute_gripper_coordinate
	 = the_robot->reply_package.arm.pf_def.gripper_coordinate;

	 }

	 if ((desired_absolute_gripper_coordinate > goal_position) || (node_counter
	 < min_node_counter)) {
	 desired_absolute_gripper_coordinate -= position_increment;
	 the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
	 = -position_increment;
	 } else {
	 return false;
	 }
	 */
	return true;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
