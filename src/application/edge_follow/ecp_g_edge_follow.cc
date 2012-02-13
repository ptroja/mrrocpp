/*!
 * @file
 * @brief File contains ecp_generator class definition of unknown contour following application.
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup edge_follow
 */

#include <cstdio>
#include <fstream>
#include <iostream>
#include <ctime>
#include <unistd.h>
#include <cmath>

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_edge_follow.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////
//
// 			y_edge_follow_force_generator
//
// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // ///////////////////

y_edge_follow_force::y_edge_follow_force(common::task::task& _ecp_task, int step) :
		teach_in(_ecp_task), step_no(step), tool_frame(0.0, 0.0, 0.25)
{
	generator_name = ecp_mp::generator::EDGE_FOLLOW;
}

bool y_edge_follow_force::first_step()
{

	std::cout << "y_edge_follow_force" << node_counter << std::endl;

	double delta[6];
	for (int i = 0; i < 6; i++)
		delta[i] = 0.0;

	ecp_t.cc_m["swarm 1"] = 5;
	//	int a = boost::any_cast <int>(ecp_t.cc_m["swarm 1"]);

	ecp_t.cc_m["swarm i swarm i swarm i swarm i swarm "] = (std::string) "sdsadsa";
	std::cout << "pupa" << boost::any_cast <int>(ecp_t.cc_m["swarm 1"])
			<< boost::any_cast <std::string>(ecp_t.cc_m["swarm i swarm i swarm i swarm i swarm "]) << std::endl;
	create_pose_list_head(emptyps, 0.0, delta, 2);

	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = ARM_DEFINITION; // arm - ORYGINAL
	the_robot->ecp_command.set_type = ARM_DEFINITION | ROBOT_MODEL_DEFINITION;
	//	the_robot->ecp_command.set_type = ARM_DEFINITION;
	the_robot->ecp_command.robot_model.type = lib::TOOL_FRAME;
	the_robot->ecp_command.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.set_arm_type = lib::PF_VELOCITY;
//	the_robot->ecp_command.get_arm_type = lib::FRAME;
	the_robot->ecp_command.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.interpolation_type = lib::TCIM;
	the_robot->ecp_command.motion_steps = step_no;
	the_robot->ecp_command.value_in_step_no = step_no - 2;

	the_robot->ecp_command.robot_model.tool_frame_def.tool_frame = tool_frame;

	for (int i = 0; i < 3; i++) {
		the_robot->ecp_command.arm.pf_def.inertia[i] = lib::FORCE_INERTIA;
		the_robot->ecp_command.arm.pf_def.inertia[i + 3] = lib::TORQUE_INERTIA;
	}

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = 0;
		the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[i] = 0;
		//	the_robot->EDP_data.ECPtoEDP_reciprocal_damping[i] = 0.0;
		the_robot->ecp_command.arm.pf_def.behaviour[i] = lib::UNGUARDED_MOTION;
	}

	the_robot->ecp_command.arm.pf_def.reciprocal_damping[0] = lib::FORCE_RECIPROCAL_DAMPING;
	the_robot->ecp_command.arm.pf_def.behaviour[0] = lib::CONTACT;
	// Sila dosciku do rawedzi
	the_robot->ecp_command.arm.pf_def.force_xyz_torque_xyz[0] = 4;

	return true;
}
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
bool y_edge_follow_force::next_step()
{
	// static int count;
	// struct timespec start[9];
	if (check_and_null_trigger()) {
		return false;
	}

	std::cout << "y_edge_follow_force" << node_counter << std::endl;

	// 	wstawienie nowego przyrostu pozyji do przyrostowej trajektorii ruchu do zapisu do pliku
	lib::Homog_matrix tmp_matrix(the_robot->reply_package.arm.pf_def.arm_frame);

	// tablice pomocnicze do utworzenia przyrostowej trajektorii ruchu do zapisu do pliku
	lib::Xyz_Euler_Zyz_vector inc_delta, tmp_delta;

	tmp_matrix.get_xyz_euler_zyz(inc_delta);

	for (int i = 0; i < 6; i++)
		inc_delta[i] = -inc_delta[i];

	tmp_matrix = the_robot->reply_package.arm.pf_def.arm_frame;
	tmp_matrix.get_xyz_euler_zyz(tmp_delta);

	for (int i = 0; i < 6; i++)
		inc_delta[i] += tmp_delta[i];

	double inc_delta_vector[6];
	inc_delta.to_table(inc_delta_vector);

	insert_pose_list_element(emptyps, 0.0, inc_delta_vector, 2);

	// wyznaczenie nowej macierzy referencyjnej i predkosci ruchu

	the_robot->ecp_command.instruction_type = lib::SET_GET;

	for (std::size_t i = 0; i < lib::MAX_SERVOS_NR; i++) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = 0.0;
	}

	// sprowadzenie sil do ukladu kisci
	lib::Ft_v_vector force_torque(the_robot->reply_package.arm.pf_def.force_xyz_torque_xyz);

	double wx = force_torque[0];
	double wy = force_torque[1];

	double v = hypot(wx, wy);

	if (v != 0.0) {

		lib::Homog_matrix basic_rot_frame;
		lib::Homog_matrix ex_rot_frame;
		double s_alfa = wy / v;
		double c_alfa = wx / v;

		the_robot->ecp_command.arm.pf_def.arm_coordinates[1] = 0.002 * v;
		//     the_robot->ecp_command.arm.pf_def.arm_coordinates[1] = -0.00;
		//	the_robot->EDP_data.ECPtoEDP_position_velocity[1] = 0.0;

		// basic_rot_frame = lib::Homog_matrix(c_alfa, s_alfa, 0.0,	-s_alfa, c_alfa, 0.0,	0.0, 0.0, 1,	0.0, 0.0, 0.0);
		basic_rot_frame = lib::Homog_matrix(c_alfa, -s_alfa, 0.0, 0.0, s_alfa, c_alfa, 0.0, 0.0, 0.0, 0.0, 1, 0.0);

		// dodatkowa macierz obracajaca kierunek wywieranej sily tak aby stabilizowac jej wartosc
		double alfa_r = 0.2 * (v - 4);
		double s_alfa_r = sin(alfa_r);
		double c_alfa_r = cos(alfa_r);

		// ex_rot_frame = lib::Homog_matrix(c_alfa_r, s_alfa_r, 0.0,	-s_alfa_r, c_alfa_r, 0.0,	0.0, 0.0, 1,	0.0, 0.0, 0.0);
		ex_rot_frame = lib::Homog_matrix(c_alfa_r, -s_alfa_r, 0.0, 0.0, s_alfa_r, c_alfa_r, 0.0, 0.0, 0.0, 0.0, 1, 0.0);

		// obrocenie pierwotnej macierzy
		basic_rot_frame = basic_rot_frame * ex_rot_frame;

		//		basic_rot_frame = !basic_rot_frame;

		tool_frame = tool_frame * basic_rot_frame;
		// basic_rot_frame.set_translation_vector(0, 0, 0.25);

		the_robot->ecp_command.robot_model.tool_frame_def.tool_frame = tool_frame;

		//	ECPtoEDP_ref_frame.get_frame_tab(the_robot->EDP_data.ECPtoEDP_reference_frame);

		/*
		 the_robot->EDP_data.ECPtoEDP_reference_frame[0][0] = c_alfa;
		 the_robot->EDP_data.ECPtoEDP_reference_frame[0][1] = s_alfa;

		 the_robot->EDP_data.ECPtoEDP_reference_frame[1][0] = -s_alfa;
		 the_robot->EDP_data.ECPtoEDP_reference_frame[1][1] = c_alfa;
		 */

		printf("sensor: x: %+ld, y: %+ld, v:%+ld, %f\n", lround(wx), lround(wy), lround(v), atan2(s_alfa, c_alfa)
				* (180.0 / M_PI));
	}

	return true;

}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
