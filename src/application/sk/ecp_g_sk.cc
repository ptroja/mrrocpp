/**
 * @file generator/ecp_g_force.cc
 * @brief ECP force generators
 * - class declaration
 * @author yoyek
 * @date 01.01.2002
 *
 * $URL: https://segomo.elka.pw.edu.pl/svn/mrrocpp/base/trunk/src/ecp/common/generator/ecp_g_force.cc $
 * $LastChangedRevision: 3198 $
 * $LastChangedDate: 2009-12-16 23:17:30 +0100 (Wed, 16 Dec 2009) $
 * $LastChangedBy: yoyek $
 */

// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - force methods
// Funkcje do tworzenia procesow ECP z wykorzystaniem sily
//
// Ostatnia modyfikacja: 2004r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <time.h>
#include <unistd.h>
#include <math.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "application/sk/ecp_g_sk.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_edge_follow_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////


y_edge_follow_force::y_edge_follow_force(common::task::task& _ecp_task,
		int step) :
	teach_in(_ecp_task), step_no(step), tool_frame(0.0, 0.0, 0.25) {
}

bool y_edge_follow_force::first_step() {
	for (int i = 0; i < 6; i++)
		delta[i] = 0.0;

	create_pose_list_head(emptyps, 0.0, delta, 2);

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION
			| ROBOT_MODEL_DEFINITION;
	//	the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.set_arm_type = lib::PF_VELOCITY;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.interpolation_type = lib::TCIM;
	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	tool_frame.get_frame_tab(
			the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame);

	for (int i = 0; i < 3; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i]
				= FORCE_INERTIA;
		the_robot->ecp_command.instruction.arm.pf_def.inertia[i + 3]
				= TORQUE_INERTIA;
	}

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i]
				= 0;
		//	the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0.0;
		the_robot->ecp_command.instruction.arm.pf_def.behaviour[i]
				= lib::UNGUARDED_MOTION;
	}

	the_robot->ecp_command.instruction.arm.pf_def.reciprocal_damping[0]
			= FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.instruction.arm.pf_def.behaviour[0] = lib::CONTACT;
	// Sila dosciku do rawedzi
	the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[0] = 4;

	return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_edge_follow_force::next_step() {
	// static int count;
	// struct timespec start[9];
	if (check_and_null_trigger()) {
		return false;
	}

	// 	wstawienie nowego przyrostu pozyji do przyrostowej trajektorii ruchu do zapisu do pliku
	lib::Homog_matrix tmp_matrix(the_robot->reply_package.arm.pf_def.arm_frame);

	// tablice pomocnicze do utworzenia przyrostowej trajektorii ruchu do zapisu do pliku
	lib::Xyz_Euler_Zyz_vector inc_delta, tmp_delta;

	tmp_matrix.get_xyz_euler_zyz(inc_delta);

	for (int i = 0; i < 6; i++)
		inc_delta[i] = -inc_delta[i];

	tmp_matrix.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
	tmp_matrix.get_xyz_euler_zyz(tmp_delta);

	for (int i = 0; i < 6; i++)
		inc_delta[i] += tmp_delta[i];

	double inc_delta_vector[6];
	inc_delta.to_table(inc_delta_vector);

	insert_pose_list_element(emptyps, 0.0, inc_delta_vector, 2);

	// wyznaczenie nowej macierzy referencyjnej i predkosci ruchu

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	for (int i = 0; i < MAX_SERVOS_NR; i++) {
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0.0;
	}

	// sprowadzenie sil do ukladu kisci
	lib::Ft_v_vector force_torque(
			the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

	double wx = force_torque[0];
	double wy = force_torque[1];

	double v = hypot(wx, wy);

	if (v != 0.0) {
		double s_alfa = wy / v;
		double c_alfa = wx / v;

		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1]
				= 0.002 * v;
		//     the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[1] = -0.00;
		//	the_robot->EDP_data.ECPtoEDP_position_velocity[1] = 0.0;

		// basic_rot_frame = lib::Homog_matrix(c_alfa, s_alfa, 0.0,	-s_alfa, c_alfa, 0.0,	0.0, 0.0, 1,	0.0, 0.0, 0.0);
		basic_rot_frame = lib::Homog_matrix(c_alfa, -s_alfa, 0.0, 0.0, s_alfa,
				c_alfa, 0.0, 0.0, 0.0, 0.0, 1, 0.0);

		// dodatkowa macierz obracajaca kierunek wywieranej sily tak aby stabilizowac jej wartosc
		double alfa_r = 0.2 * (v - 4);
		double s_alfa_r = sin(alfa_r);
		double c_alfa_r = cos(alfa_r);

		// ex_rot_frame = lib::Homog_matrix(c_alfa_r, s_alfa_r, 0.0,	-s_alfa_r, c_alfa_r, 0.0,	0.0, 0.0, 1,	0.0, 0.0, 0.0);
		ex_rot_frame = lib::Homog_matrix(c_alfa_r, -s_alfa_r, 0.0, 0.0,
				s_alfa_r, c_alfa_r, 0.0, 0.0, 0.0, 0.0, 1, 0.0);

		// obrocenie pierwotnej macierzy
		basic_rot_frame = basic_rot_frame * ex_rot_frame;

		//		basic_rot_frame = !basic_rot_frame;

		tool_frame = tool_frame * basic_rot_frame;
		// basic_rot_frame.set_translation_vector(0, 0, 0.25);

		tool_frame.get_frame_tab(
				the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame);

		//	ECPtoEDP_ref_frame.get_frame_tab(the_robot->EDP_data.ECPtoEDP_reference_frame);

		/*
		 the_robot->EDP_data.ECPtoEDP_reference_frame[0][0] = c_alfa;
		 the_robot->EDP_data.ECPtoEDP_reference_frame[0][1] = s_alfa;

		 the_robot->EDP_data.ECPtoEDP_reference_frame[1][0] = -s_alfa;
		 the_robot->EDP_data.ECPtoEDP_reference_frame[1][1] = c_alfa;
		 */

		printf("sensor: x: %+ld, y: %+ld, v:%+ld, %f\n", lround(wx),
				lround(wy), lround(v), atan2(s_alfa, c_alfa)
						* DEGREES_TO_RADIANS);
	}

	return true;

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
