/*
 * generator/ecp_g_mboryn.cpp
 *
 *  Created on: Dec 11, 2009
 *      Author: mboryn
 */

#include "ecp_g_mboryn.h"

#include <iostream>

using namespace std;

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
	vsp_fradia = sensor_m[lib::SENSOR_CVFRADIA];

	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
	the_robot->ecp_command.instruction.motion_steps = 25;
	the_robot->ecp_command.instruction.value_in_step_no = 25 - 1;

	is_calculated = false;
	kkk = 0;
	printf("ecp_g_mboryn::first_step()\n"); fflush(stdout);

	return true;
}

bool ecp_g_mboryn::next_step()
{
	printf("ecp_g_mboryn::next_step() begin: %d\n", kkk); fflush(stdout);

	if(++kkk > 5){
		printf("ecp_g_mboryn::next_step() end\n"); fflush(stdout);
		return false;
	}

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
	the_robot->ecp_command.instruction.get_type = ARM_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
	the_robot->ecp_command.instruction.set_type = ARM_DV;
	the_robot->ecp_command.instruction.set_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
	the_robot->ecp_command.instruction.motion_steps = 25;
	the_robot->ecp_command.instruction.value_in_step_no = 25 - 1;

	lib::Homog_matrix currentFrame;
	double gripper_coordinate;
	currentFrame.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);
	gripper_coordinate = the_robot->reply_package.arm.pf_def.gripper_coordinate;
	cout<<"\ncurrentFrame:\n"<<currentFrame<<endl;

	for(int i=0; i<MAX_SERVOS_NR; ++i){
		printf("arm_coordinates[%d]: %g,\t", i, the_robot->reply_package.arm.pf_def.arm_coordinates[i]);
	}
	printf("\n"); fflush(stdout);

	currentFrame.get_frame_tab(the_robot->ecp_command.instruction.arm.pf_def.arm_frame);
	the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate = gripper_coordinate;

	lib::VSP_REPORT vsp_report = vsp_fradia->from_vsp.vsp_report;
	if (vsp_report == lib::VSP_REPLY_OK) {
		printf("vsp_report == lib::VSP_REPLY_OK\n"); fflush(stdout);

		int u[3];
		u[0] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.x;
		u[1] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.y;
		u[2] = vsp_fradia->from_vsp.comm_image.sensor_union.object_tracker.z;

		printf("ux: %d\t", u[0]);
		printf("uy: %d\t", u[1]);
		printf("uz: %d\n", u[2]);
		fflush(stdout);
	}
	else{
		printf("vsp_report != lib::VSP_REPLY_OK\n"); fflush(stdout);
	}

	/*if(!is_calculated){
		double distance = 0;

		for(int i=0; i<3; ++i){
			distance += target_xyz
		}
	}*/

	return true;
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
