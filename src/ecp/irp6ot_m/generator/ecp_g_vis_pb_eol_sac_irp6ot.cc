///////////////////////////////////////////////////////////
//  generator/ecp_g_vis_pb_eol_sac_irp6ot.cpp
//  Implementation of the Class generator/ecp_g_vis_pb_eol_sac_irp6ot
//  Created on:      04-sie-2008 14:25:57
//  Original author: tkornuta
///////////////////////////////////////////////////////////

/*!
 * \file generator/ecp_g_vis_pb_eol_sac_irp6ot.cc
 * \brief Class implementing PB-EOL-SAC algorithm.
 * - methods definition
 * \author Maciej Staniak
 * \date 20.08.2008
 */

#include <stdio.h>
#include <math.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6_on_track/generator/ecp_g_vis_pb_eol_sac_irp6ot.h"


namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace generator {

ecp_vis_pb_eol_sac_irp6ot::ecp_vis_pb_eol_sac_irp6ot(common::task::task& _ecp_task, int step) : common::generator::ecp_visual_servo(_ecp_task){

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

	gain[0]=ecp_t.config.value<double>("gain0");
	gain[1]=ecp_t.config.value<double>("gain1");
	gain[2]=ecp_t.config.value<double>("gain2");
	gain[3]=ecp_t.config.value<double>("gain3");
	gain[4]=ecp_t.config.value<double>("gain4");
	gain[5]=ecp_t.config.value<double>("gain5");

	x2g=ecp_t.config.value<double>("x2g");

}

bool ecp_vis_pb_eol_sac_irp6ot::next_step_without_constraints(){

	 the_robot->ecp_command.instruction.set_type = ARM_DEFINITION;
	the_robot->ecp_command.instruction.instruction_type = lib::SET_GET;

	//G_Tx_G2.set_from_xyz_euler_zyz( 0,0,0, 0.002, 1.481+0.03, 2.341);	//jesli chwytamy po przekatnej
	G_Tx_G2.set_from_xyz_euler_zyz( 0,0,0, 0.00, 1.57, 3.141); // jesli chwytak na plasko



	if (node_counter==1)
	{
		vsp_vis_sac->base_period=1;
		vsp_vis_sac->current_period=1;
	}

	if (node_counter==1)
	{
		std::cout << std::endl << std::endl << std::endl << std::endl
				<< std::endl;

		O_Tx_E.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame); // zarem
		std::cout << "YYY " << O_Tx_E << std::endl;

		O_Tx_E.get_xyz_angle_axis(O_r_E[0]);
		O_Tx_E=O_Tx_E*!G_Tx_G2; //tool zamiast

		std::cout << "MMM " << node_counter << std::endl;
		std::cout << "YYY " << O_Tx_E << std::endl;

		for (int i=0; i<6; i++)
		{

			O_r_Ep[0][i]=O_r_E[0][i];
			O_r_Ep[1][i]=O_r_E[0][i];
			O_r_Ep[2][i]=O_r_E[0][i];

			O_r_Ep_d[i][1]=0;
			O_r_Ep_d[i][2]=0;
		}


		std::cout << node_counter
				<<"------------------------------------------------------------------"
				<< std::endl;
	}

	//SAC
	C_Tx_G.set_from_xyz_rpy(vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G[0],
			vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G[1],
			-vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G[2],
			vsp_vis_sac->image.sensor_union.vis_sac.frame_E_r_G[5], 0, 0);
		//	0, 0, -0.1);

#if 0
	//podjazd gdy sie nie ruszamy
	if (fabs(O_r_G[1][0]-O_r_G[0][0])<=0.02 && fabs(O_r_G[1][1]-O_r_G[0][1])
			<=0.02) //0.007
	{
		steps2switch++;

	}
	else
	{
		x2g=x2g_begin;
		steps2switch=0;
	}

	if (steps2switch>15) // to approach 15
		vis_phase=1;

	if (vis_phase)
	{
		x2g+=0.0; //0.002; //0.001; //NIE PODJEZDZAMY
	}

	if (x2g>=0.0)
	{
		x2g=0.0; //0.02
		//return true; //to avoid graspin'
		//return false;	//to grasp
	}
#endif


	G_Tx_S.set_from_xyz_rpy(x2g, 0, 0, 0, 0, 0);
	O_Tx_E.set_from_frame_tab(the_robot->reply_package.arm.pf_def.arm_frame);

	O_Tx_E=O_Tx_E*!G_Tx_G2; //zmiana orientacji (teraz tool)

	O_Tx_E.get_xyz_angle_axis(O_r_E[0]);

	//SAC
	O_Tx_C.set_from_xyz_rpy( 0.950+0.058, //-0.09,
			0.000-0.06, 0.265+0.900+0.075-0.105, 0, 0, 0);
	O_Tx_G=O_Tx_C*C_Tx_G;
	O_Tx_G=O_Tx_G*G_Tx_S; //skrot myslowy
	O_Tx_G.get_xyz_angle_axis(O_r_G[0]);

	O_eps_EG_norm=0.0;
	for (int i=0; i<6; i++)
		{
			O_eps_EG[0][i]=(O_r_G[0][i]-O_r_E[0][i]);
			O_eps_EG_norm+=O_eps_EG[0][i]*O_eps_EG[0][i];
			O_eps_E[0][i]=gain[i]*O_eps_EG[0][i];
		}

	for (int i=0; i<6; i++)
	{
		O_r_Ep[0][i]=O_r_E[0][i]+O_eps_E[0][i];
	}

	if (node_counter==1)
	{
		for (int i=0; i<6; i++)
		{
			O_r_Ep[0][i]=O_r_E[0][i];
		}
	}

	// TODO: this return was missing
	return false;
}

#if 0
void ecp_vis_pb_eol_sac_irp6ot::limit_step(){
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


	O_Tx_Ep=O_Tx_Ep*G_Tx_G2; //zmiana orientacji teraz tool

	O_Tx_Ep.get_xyz_angle_axis(O_r_Ep[0]);

	std::cout << "ECP Ep: ";

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

bool ecp_vis_pb_eol_sac_irp6ot::first_step(void){
/*
	vsp_vis_sac = sensor_m[lib::SENSOR_CAMERA_SA];

	idle_step_counter = 1;
	vsp_vis_sac->base_period=0; //1
	vsp_vis_sac->current_period=0; //MAC7
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
	the_robot->ecp_command.instruction.set_robot_model_type = lib::TOOL_FRAME; //TOOL_XYZ_EULER_ZYZ; //TOOL_FRAME;
	the_robot->ecp_command.instruction.get_robot_model_type = lib::TOOL_FRAME; //TOOL_XYZ_EULER_ZYZ;  //TOOL_FRAME;
	the_robot->ecp_command.instruction.interpolation_type= lib::MIM;
	the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;

	the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
	the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] = 0;
	}

	O_eps_EG_norm=10;

	return true;
	*/
}

} // namespace generator
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp


