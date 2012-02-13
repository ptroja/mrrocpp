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

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_tff_gripper_approach.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

tff_gripper_approach::tff_gripper_approach(common::task::task& _ecp_task, int step) :
		common::generator::generator(_ecp_task), speed(0.0), motion_time(1), force_level(-10.0), step_no(step)
{
	generator_name = ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH;
}

void tff_gripper_approach::configure(double l_speed, unsigned int l_motion_time, double l_force_level)
{
	speed = l_speed;
	motion_time = l_motion_time;
	force_level = l_force_level;
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
	the_robot->ecp_command.robot_model.tool_frame_def.tool_frame = tool_frame;

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	//	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::TOOL_FRAME;
	the_robot->ecp_command.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.set_arm_type = lib::PF_VELOCITY;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::RELATIVE;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = td.internode_step_no;
	the_robot->ecp_command.value_in_step_no = td.value_in_step_no;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = 0;
	}

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.inertia[i] = 0;
	}

	the_robot->ecp_command.arm.pf_def.inertia[2] = lib::FORCE_INERTIA / 4;
	the_robot->ecp_command.arm.pf_def.arm_coordinates[2] = speed;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
		//		the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0;
	}

	the_robot->ecp_command.arm.pf_def.behaviour[2] = lib::GUARDED_MOTION;
	the_robot->ecp_command.arm.pf_def.reciprocal_damping[2] = lib::FORCE_RECIPROCAL_DAMPING / 2;

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_gripper_approach::next_step()
{
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	static int high_force = 0;

	if (node_counter == 1) {

	} else {

		std::cout << the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[2] << std::endl;

		if (node_counter > motion_time) {
			return false;
		}
		if ((force_level > 0) && (the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz[2] > force_level)) {
			high_force++;
		} else {
			high_force = 0;
		}
		if (high_force > 10) {
			return false;
		}
	}

	return true;
}

void tff_gripper_approach::conditional_execution()
{
	ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type beh;

	switch ((ecp_mp::generator::tff_gripper_approach::communication_type) ecp_t.mp_command.ecp_next_state.variant)
	{
		case ecp_mp::generator::tff_gripper_approach::behaviour_specification:
			ecp_t.mp_command.ecp_next_state.sg_buf.get(beh);
			configure(beh.speed, beh.motion_time, beh.force_level);
			break;
		case ecp_mp::generator::tff_gripper_approach::no_data:
			break;
		default:
			break;
	}
	Move();
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
