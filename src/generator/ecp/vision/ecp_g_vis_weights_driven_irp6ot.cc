///////////////////////////////////////////////////////////
//  generator/ecp_g_vis_weights_driven_irp6ot.cpp
//  Implementation of the Class generator/ecp_g_vis_weights_driven_irp6ot
//  Created on:      04-sie-2008 14:25:49
//  Original author: tkornuta
///////////////////////////////////////////////////////////

/*!
 * \file generator/ecp_g_vis_weight_driven_irp6ot.h
 * \brief Class implementing switching algorithm.
 * - methods definiotion
 * \author Maciej Staniak
 * \date 20.08.2008
 */

#include <cstdio>
#include <cmath>
#include <iostream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "base/ecp/irp6_on_track/generator/ecp_g_vis_weights_driven_irp6ot.h"

#include "base/ecp/irp6_on_track/generator/ecp_g_vis_pb_eol_sac_irp6ot.h"
#include "base/ecp/irp6_on_track/generator/ecp_g_vis_pb_eih_irp6ot.h"

#include "base/ecp_mp/sensor/ecp_mp_s_vis.h"
#include "base/ecp/irp6_on_track/task/ecp_t_vis_weights_driven_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

	ecp_vis_weights_driven_irp6ot::ecp_vis_weights_driven_irp6ot(common::task::task& _ecp_task, int step) : common::generator::ecp_visual_servo_manager(_ecp_task){

	measure_border_u[0]=ecp_t.config.value<double>("measure_border_u0");
	measure_border_u[1]=ecp_t.config.value<double>("measure_border_u1");
	measure_border_u[2]=ecp_t.config.value<double>("measure_border_u2");
	measure_border_u[3]=ecp_t.config.value<double>("measure_border_u3");
	measure_border_u[4]=ecp_t.config.value<double>("measure_border_u4");
	measure_border_u[5]=ecp_t.config.value<double>("measure_border_u5");

	measure_border_d[0]=ecp_t.config.value<double>("measure_border_d0");
	measure_border_d[1]=ecp_t.config.value<double>("measure_border_d1");
	measure_border_d[2]=ecp_t.config.value<double>("measure_border_d2");
	measure_border_d[3]=ecp_t.config.value<double>("measure_border_d3");
	measure_border_d[4]=ecp_t.config.value<double>("measure_border_d4");
	measure_border_d[5]=ecp_t.config.value<double>("measure_border_d5");

	d_u_max[0]=ecp_t.config.value<double>("d_u_max0");
	d_u_max[1]=ecp_t.config.value<double>("d_u_max1");
	d_u_max[2]=ecp_t.config.value<double>("d_u_max2");
	d_u_max[3]=ecp_t.config.value<double>("d_u_max3");
	d_u_max[4]=ecp_t.config.value<double>("d_u_max4");
	d_u_max[5]=ecp_t.config.value<double>("d_u_max5");

	d2_u_max[0]=ecp_t.config.value<double>("d2_u_max0");
	d2_u_max[1]=ecp_t.config.value<double>("d2_u_max1");
	d2_u_max[2]=ecp_t.config.value<double>("d2_u_max2");
	d2_u_max[3]=ecp_t.config.value<double>("d2_u_max3");
	d2_u_max[4]=ecp_t.config.value<double>("d2_u_max4");
	d2_u_max[5]=ecp_t.config.value<double>("d2_u_max5");

	O_weight__SAC=ecp_t.config.value<double>("O_weight__SAC");
	C_weight__EIH=ecp_t.config.value<double>("C_weight__EIH");
	f_weight__EIH=ecp_t.config.value<double>("f_weight__EIH");

}

void ecp_vis_weights_driven_irp6ot::initalize_switching_parameters(){

}

bool ecp_vis_weights_driven_irp6ot::next_step_without_constraints(){

	//generator/ecp_g_vis_pb_eol_sac_irp6ot pbeolsac(*_ecp_taskw, 4);
	//generator/ecp_g_vis_pb_eol_sac_irp6ot pbeolsac();
//		generator/ecp_g_vis_weights_driven_irp6ot ynrlg(*this, 4);
std::cout << "N: " << node_counter << " " << pbeolsac->node_counter << std::endl;
	//pbeolsac->next_step_without_constraints();
	pbeih->next_step_without_constraints();

	for (int i=0; i<6; i++)
	{
		//O_r_Ep[0][i]=pbeolsac->O_r_Ep[0][i];
		O_r_Ep[0][i]=pbeih->O_r_Ep[0][i];
	}

	for (int i=0; i<6; i++)
	{
		//O_r_E[0][i]=pbeolsac->O_r_E[0][i];
		O_r_E[0][i]=pbeih->O_r_E[0][i];
	}

	if (node_counter==1)
	{
		for (int i=0; i<6; i++)
		{
			//O_r_Ep[0][i]=pbeolsac->O_r_E[0][i];
			O_r_Ep[0][i]=pbeih->O_r_E[0][i];
		}
	}

	pbeolsac->node_counter++;
	pbeih->node_counter++;
	ibeih->node_counter++;

	// TODO: this return was missing
	return false;
}

#if 0
void ecp_vis_weights_driven_irp6ot::limit_step(){
	// roznica w kroku -> docelowo predkosc
	for (int i=0; i<6; i++)
		O_r_Ep_d[0][i]=O_r_Ep[0][i]-O_r_E[0][i];

	// roznica w 2 krokach -> docelowo przyspieszenie
	for (int i=0; i<6; i++)
		O_r_Ep_d2[0][i]=O_r_Ep_d[0][i]-O_r_Ep_d[1][i];

	//ograniczenie przyspieszenia (opoznienie zalatwia regulator proporcjonalny)
	for (int i=0; i<6; i++)
	{
		if ((fabs(O_r_Ep_d2[0][i])>=d2_u_max[i]) && (fabs(O_r_Ep_d[0][i])
				>=fabs(O_r_Ep_d[1][i])))
		{
			if (O_r_Ep_d[0][i]>=0)
			{
				O_r_Ep_d[0][i]=O_r_Ep_d[1][i]+d2_u_max[i];
				O_r_Ep[0][i]=O_r_E[0][i]+O_r_Ep_d[0][i];
			}
			else if (O_r_Ep_d[0][i]<0)
			{
				O_r_Ep_d[0][i]=O_r_Ep_d[1][i]-d2_u_max[i];
				O_r_Ep[0][i]=O_r_E[0][i]+O_r_Ep_d[0][i];
			}
		}
	}

	O_Tx_Ep.set_from_xyz_angle_axis(lib::Xyz_Angle_Axis_vector(O_r_Ep[0]));

	// ------------przepisanie wartosci-----
	for (int i=0; i<6; i++)
	{
		O_r_Ep[2][i]=O_r_Ep[1][i];
		O_r_Ep[1][i]=O_r_Ep[0][i];
		O_r_Ep_d[i][2]=O_r_Ep_d[i][1];
		O_r_Ep_d[i][1]=O_r_Ep_d[i][0];

		//O_r_G[1][i]=O_r_G[0][i];
		//C_r_G[1][i]=C_r_G[0][i];
	}


	//O_Tx_Ep=O_Tx_Ep*pbeolsac->G_Tx_G2;
	O_Tx_Ep=O_Tx_Ep*pbeih->G_Tx_G2;

	O_Tx_Ep.get_xyz_angle_axis(O_r_Ep[0]);

	std::cout << "ecp Ep: ";

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = O_r_Ep[0][i];
		std::cout << O_r_Ep[0][i] << " ";
	}

	std::cout << std::endl;
	/*
	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = the_robot->reply_package.arm.pf_def.arm_coordinates[i];
	}
	*/


		the_robot->ecp_command.instruction.arm.pf_def.gripper_coordinate
			=the_robot->reply_package.arm.pf_def.gripper_coordinate;

}
#endif

bool ecp_vis_weights_driven_irp6ot::first_step(void){
/*
	pbeolsac->vsp_vis_sac = sensor_m[lib::SENSOR_CAMERA_SA];
	pbeolsac->node_counter = 1;
	pbeolsac->idle_step_counter = 1;

	pbeih->vsp_vis_sac = sensor_m[lib::SENSOR_CAMERA_SA];
	pbeih->node_counter = 1;
	pbeih->idle_step_counter = 1;

	ibeih->vsp_vis_sac = sensor_m[lib::SENSOR_CAMERA_SA];
	ibeih->node_counter = 1;
	ibeih->idle_step_counter = 1;

	idle_step_counter = 1;
	pbeolsac->vsp_vis_sac->base_period=0; //1
	pbeolsac->vsp_vis_sac->current_period=0; //MAC7
	//vsp_force_irp6p->base_period=1;
	//td.interpolation_node_no = 1; //potrzebne? MAC7
	td.internode_step_no = 50; //step_no; 40
	td.value_in_step_no = td.internode_step_no - 5; //2 //10

	//TOOL


	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][0]=1;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][0]=0;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][0]=0;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][3]=0;

	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][1]=0;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][1]=1;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][1]=0;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][3]=0;

	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[0][2]=0;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[1][2]=0;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][2]=1;
	the_robot->ecp_command.instruction.robot_model.tool_frame_def.tool_frame[2][3]=0.25;

	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;
	the_robot->ecp_command.instruction.get_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.set_type = ROBOT_MODEL_DEFINITION;
	the_robot->ecp_command.instruction.set_arm_type = lib::XYZ_ANGLE_AXIS;
	the_robot->ecp_command.instruction.get_arm_type = lib::FRAME;
	the_robot->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME;
	the_robot->ecp_command.instruction.interpolation_type= lib::MIM;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;

	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
		the_robot->ecp_command.instruction.arm.pf_def.force_xyz_torque_xyz[i] = 0;
	}

	O_eps_EG_norm=10;

	return true;
	*/
}

/*
bool ecp_vis_weights_driven_irp6ot::next_step(void){

	return false;
}
*/
} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


