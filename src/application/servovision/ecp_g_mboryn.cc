/*
 * generator/ecp_g_mboryn.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_g_mboryn.h"

namespace mrrocpp {

namespace ecp {

namespace irp6ot {

namespace generator {

ecp_g_mboryn::ecp_g_mboryn(mrrocpp::ecp::common::task::task & _ecp_task)
//: mrrocpp::ecp::common::generator::generator(_ecp_task)
:generator(_ecp_task)
{
	target_xyz[0] = 0;
	target_xyz[1] = 0;
	target_xyz[2] = 0;
}

ecp_g_mboryn::~ecp_g_mboryn() {
}

bool ecp_g_mboryn::first_step()
{
	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
	the_robot->ecp_command.instruction.motion_steps = 25;
	the_robot->ecp_command.instruction.value_in_step_no = 25 - 1;

	//memcpy(next_position, the_robot->ecp_command.instruction.current_XYZ_AA_arm_coordinates, 6 * sizeof(double));

	is_calculated = false;

	printf("ecp_g_mboryn::first_step()\n"); fflush(stdout);

	/*printf("next_position: ");
	for(int i=0; i<6; ++i){
		printf("%lg; ", next_position[i]);
	}
	printf("\n"); fflush(stdout);*/

	return true;
}

bool ecp_g_mboryn::next_step()
{
	/*if(!is_calculated){
		double distance = 0;

		for(int i=0; i<3; ++i){
			distance += target_xyz
		}
	}*/

	printf("ecp_g_mboryn::next_step()\n"); fflush(stdout);

	return false;
}

void ecp_g_mboryn::set_target(double xyz[3]){
	target_xyz[0] = xyz[0];
	target_xyz[1] = xyz[1];
	target_xyz[2] = xyz[2];
}

} // namespace mrrocpp

} // namespace irp6ot

} // namespace ecp

} // namespace mrrocpp
