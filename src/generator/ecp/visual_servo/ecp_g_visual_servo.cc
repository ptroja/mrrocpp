///////////////////////////////////////////////////////////
//  ecp_visual_servo.cpp
//  Implementation of the Class ecp_visual_servo
//  Created on:      04-sie-2008 14:21:20
//  Original author: tkornuta
///////////////////////////////////////////////////////////

/*!
 * \file ecp_visual_servo.cc
 * \brief Abstract class as a pattern for implementing any visual servo.
 * - methods definition
 * \author tkornuta/mstaniak
 * \date 04.08.2008
 */

#include "base/ecp/ecp_robot.h"
#include "ecp_g_visual_servo.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

ecp_visual_servo::ecp_visual_servo(common::task::task& _ecp_task, int step) :
		common::generator::generator(_ecp_task)
{
}

ecp_visual_servo::~ecp_visual_servo()
{

}

void ecp_visual_servo::retrieve_parameters()
{

}

bool ecp_visual_servo::next_step(void)
{
	if (next_step_without_constraints()) {
		limit_step();
		return true;
	} //: if
	return false;
}

void ecp_visual_servo::set_constraints()
{

}

void ecp_visual_servo::get_constraints()
{

}

void ecp_visual_servo::set_entities()
{

}

void ecp_visual_servo::get_entities()
{

}

void ecp_visual_servo::set_opartions()
{

}

void ecp_visual_servo::get_operations()
{

}

#if 1
void ecp_visual_servo::limit_step()
{
	// roznica w kroku -> docelowo predkosc
	for (int i = 0; i < 6; i++)
		O_r_Ep_d[0][i] = O_r_Ep[0][i] - O_r_E[0][i];

	// roznica w 2 krokach -> docelowo przyspieszenie
	for (int i = 0; i < 6; i++)
		O_r_Ep_d2[0][i] = O_r_Ep_d[0][i] - O_r_Ep_d[1][i];

	//ograniczenie przyspieszenia (opoznienie zalatwia regulator proporcjonalny)
	for (int i = 0; i < 6; i++) {
		if ((fabs(O_r_Ep_d2[0][i]) >= d2_u_max[i]) && (fabs(O_r_Ep_d[0][i]) >= fabs(O_r_Ep_d[1][i]))) {
			if (O_r_Ep_d[0][i] >= 0) {
				O_r_Ep_d[0][i] = O_r_Ep_d[1][i] + d2_u_max[i];
				O_r_Ep[0][i] = O_r_E[0][i] + O_r_Ep_d[0][i];
			} else if (O_r_Ep_d[0][i] < 0) {
				O_r_Ep_d[0][i] = O_r_Ep_d[1][i] - d2_u_max[i];
				O_r_Ep[0][i] = O_r_E[0][i] + O_r_Ep_d[0][i];
			}
		}
	}
	O_Tx_Ep.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(O_r_Ep[0]));

	// ------------przepisanie wartosci-----
	for (int i = 0; i < 6; i++) {
		O_r_Ep[2][i] = O_r_Ep[1][i];
		O_r_Ep[1][i] = O_r_Ep[0][i];
		O_r_Ep_d[i][2] = O_r_Ep_d[i][1];
		O_r_Ep_d[i][1] = O_r_Ep_d[i][0];

		//O_r_G[1][i]=O_r_G[0][i];
		//C_r_G[1][i]=C_r_G[0][i];
	}

	O_Tx_Ep = O_Tx_Ep * G_Tx_G2; //zmiana orientacji teraz tool

	lib::Xyz_Angle_Axis_vector tmp_vector;
	O_Tx_Ep.get_xyz_angle_axis(tmp_vector);
	tmp_vector.to_table(O_r_Ep[0]);
	//std::cout << "ecp Ep: ";

	for (int i = 0; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = O_r_Ep[0][i];
		//	std::cout << O_r_Ep[0][i] << " ";
	}
	/*
	 std::cout << std::endl;

	 std::cout << "ecp E: ";
	 for (int i=0; i<6; i++)
	 {
	 std::cout << O_r_E1[i] << " ";
	 }

	 std::cout << std::endl;
	 */
	for (int i = 0; i < 1; i++) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = O_r_E1[i];
	}

	for (int i = 2; i < 6; i++) {
		the_robot->ecp_command.arm.pf_def.arm_coordinates[i] = O_r_E1[i];
	}

}
#endif

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
