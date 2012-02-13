/*!
 * @file
 * @brief File contains tff_rubik_face_rotate generator definition
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
#include "ecp_g_tff_rubik_face_rotate.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

tff_rubik_face_rotate::tff_rubik_face_rotate(common::task::task& _ecp_task, int step) :
		common::generator::generator(_ecp_task), step_no(step)
{
	generator_name = ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE;
}

void tff_rubik_face_rotate::configure(double l_turn_angle)
{
	turn_angle = l_turn_angle;
}

// ----------------------------------------------------------------------------------------------
// ---------------------------------    metoda	first_step -------------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_face_rotate::first_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i oreintacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// cout << "first_step" << endl;

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
	//the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::RELATIVE;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = td.internode_step_no;
	the_robot->ecp_command.value_in_step_no = td.value_in_step_no;

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = 0;
	}

	for (int i = 0; i < 3; i++) {
		the_robot->ecp_command.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		the_robot->ecp_command.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;
	}

	the_robot->ecp_command.arm.pf_def.reciprocal_damping[5] = lib::TORQUE_RECIPROCAL_DAMPING / 4;
	the_robot->ecp_command.arm.pf_def.behaviour[5] = lib::CONTACT;

	if (-0.1 < turn_angle && turn_angle < 0.1) {
		for (int i = 0; i < 6; i++)
			the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	} else {
		for (int i = 0; i < 3; i++) {
			the_robot->ecp_command.arm.pf_def.reciprocal_damping[i] = lib::FORCE_RECIPROCAL_DAMPING;
			the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::CONTACT;
		}
		for (int i = 3; i < 5; i++)
			the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;

		the_robot->ecp_command.arm.pf_def.reciprocal_damping[5] = lib::TORQUE_RECIPROCAL_DAMPING;
		the_robot->ecp_command.arm.pf_def.behaviour[5] = lib::CONTACT;
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[5] = copysign(2.5, turn_angle);
	}

	return true;
}

// ----------------------------------------------------------------------------------------------
// -----------------------------------  metoda	next_step -----------------------------------
// ----------------------------------------------------------------------------------------------

bool tff_rubik_face_rotate::next_step()
{
	// Generacja trajektorii prostoliniowej o zadany przyrost polozenia i orientacji
	// Funkcja zwraca false gdy koniec generacji trajektorii
	// Funkcja zwraca true gdy generacja trajektorii bedzie kontynuowana
	// UWAGA: dzialamy na jednoelementowej liscie robotow
	// cout << "next_step" << endl;

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji
	the_robot->ecp_command.instruction_type = lib::SET_GET;

	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu

	if (node_counter == 1) {

		if (turn_angle < -0.1 || 0.1 < turn_angle) {
			lib::Homog_matrix frame(the_robot->reply_package.arm.pf_def.arm_frame);
			lib::Xyz_Euler_Zyz_vector xyz_eul_zyz;
			frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double angle_to_move = (turn_angle / 180.0) * M_PI;
			if (xyz_eul_zyz[5] + angle_to_move < -M_PI) {
				stored_gamma = 2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				range_change = true;
			} else if (xyz_eul_zyz[5] + angle_to_move > M_PI) {
				stored_gamma = -2 * M_PI + xyz_eul_zyz[5] + angle_to_move;
				range_change = true;
			} else {
				stored_gamma = xyz_eul_zyz[5] + angle_to_move;
				range_change = false;
			}
		}
	} else {

		if (turn_angle < -0.1 || 0.1 < turn_angle) {
			lib::Homog_matrix current_frame(the_robot->reply_package.arm.pf_def.arm_frame);
			lib::Xyz_Euler_Zyz_vector xyz_eul_zyz;
			current_frame.get_xyz_euler_zyz(xyz_eul_zyz);
			double current_gamma = xyz_eul_zyz[5];
			if (!range_change) {
				if ((turn_angle < 0.0 && stored_gamma > current_gamma)
						|| (turn_angle > 0.0 && stored_gamma < current_gamma)) {
					return false;
				}
			} else {
				if ((turn_angle < 0.0 && stored_gamma < current_gamma)
						|| (turn_angle > 0.0 && stored_gamma > current_gamma)) {
					range_change = false;
				}
			}
		}

	}

	return true;
}

void tff_rubik_face_rotate::conditional_execution()
{

	switch ((ecp_mp::generator::RCSC_TURN_ANGLES) ecp_t.mp_command.ecp_next_state.variant)
	{
		case ecp_mp::generator::RCSC_CCL_90:
			configure(-90.0);
			break;
		case ecp_mp::generator::RCSC_CL_0:
			configure(0.0);
			break;
		case ecp_mp::generator::RCSC_CL_90:
			configure(90.0);
			break;
		case ecp_mp::generator::RCSC_CL_180:
			configure(180.0);
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
