///////////////////////////////////////////////////////////
//  ecp_vis_weights_driven_irp6ot.cpp
//  Implementation of the Class ecp_vis_weights_driven_irp6ot
//  Created on:      04-sie-2008 14:25:49
//  Original author: tkornuta
///////////////////////////////////////////////////////////

#include <stdio.h>
#include <math.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"

#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_vis_weights_driven_irp6ot.h"

#include "ecp/irp6_on_track/ecp_vis_pb_eol_sac_irp6ot.h"


ecp_vis_weights_driven_irp6ot::ecp_vis_weights_driven_irp6ot(ecp_task& _ecp_task, int step) : ecp_visual_servo_manager(_ecp_task){

//	_ecp_taskw=_ecp_task;

	O_weight__SAC=ecp_t.config.return_double_value("O_weight__SAC");
	C_weight__EIH=ecp_t.config.return_double_value("C_weight__EIH");
	f_weight__EIH=ecp_t.config.return_double_value("f_weight__EIH");

}



ecp_vis_weights_driven_irp6ot::~ecp_vis_weights_driven_irp6ot(){

}



void ecp_vis_weights_driven_irp6ot::initalize_switching_parameters(){

}

void ecp_vis_weights_driven_irp6ot::next_step_without_constraints(){

	//ecp_vis_pb_eol_sac_irp6ot pbeolsac(*_ecp_taskw, 4);
	//ecp_vis_pb_eol_sac_irp6ot pbeolsac();
//		ecp_vis_weights_driven_irp6ot ynrlg(*this, 4);

	for (int i=0; i<6; i++)
	{
		O_r_Ep[0][i]=0.0;
	}
	
	if (node_counter==1)
	{
		for (int i=0; i<6; i++)
		{
			O_r_Ep[0][i]=O_r_E[0][i];
		}
	}
}

void ecp_vis_weights_driven_irp6ot::entertain_constraints(){
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
	
	O_Tx_Ep.set_xyz_angle_axis(O_r_Ep[0]);
	
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

	
	O_Tx_Ep=O_Tx_Ep*G_Tx_G2;

	O_Tx_Ep.get_xyz_angle_axis(O_r_Ep[0]);
	
	std::cout << "ECP Ep: ";
	
	for (int i=0; i<6; i++)
	{
		the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = O_r_Ep[0][i];
		std::cout << O_r_Ep[0][i] << " ";	
	}
	
	std::cout << std::endl;
	/*
	for (int i=0; i<6; i++)
	{
		the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = the_robot->EDP_data.current_XYZ_AA_arm_coordinates[i];
	}
	*/

	
		the_robot->EDP_data.next_gripper_coordinate
			=the_robot->EDP_data.current_gripper_coordinate;
			
}

bool ecp_vis_weights_driven_irp6ot::first_step(void){

	vsp_vis_sac = sensor_m[SENSOR_CAMERA_SA];

	idle_step_counter = 1;
	vsp_vis_sac->base_period=0; //1
	vsp_vis_sac->current_period=0; //MAC7
	//vsp_force_irp6p->base_period=1;
	//td.interpolation_node_no = 1; //potrzebne? MAC7
	td.internode_step_no = 50; //step_no; 40
	td.value_in_step_no = td.internode_step_no - 5; //2 //10

	//TOOL


	the_robot->EDP_data.next_tool_frame[0][0]=1;
	the_robot->EDP_data.next_tool_frame[1][0]=0;
	the_robot->EDP_data.next_tool_frame[2][0]=0;
	the_robot->EDP_data.next_tool_frame[0][3]=0;

	the_robot->EDP_data.next_tool_frame[0][1]=0;
	the_robot->EDP_data.next_tool_frame[1][1]=1;
	the_robot->EDP_data.next_tool_frame[2][1]=0;
	the_robot->EDP_data.next_tool_frame[1][3]=0;

	the_robot->EDP_data.next_tool_frame[0][2]=0;
	the_robot->EDP_data.next_tool_frame[1][2]=0;
	the_robot->EDP_data.next_tool_frame[2][2]=1;
	the_robot->EDP_data.next_tool_frame[2][3]=0.25; 

	the_robot->EDP_data.instruction_type = SET_GET;
	the_robot->EDP_data.get_type = ARM_DV;
	the_robot->EDP_data.set_type = RMODEL_DV;
	the_robot->EDP_data.set_arm_type = XYZ_ANGLE_AXIS;
	the_robot->EDP_data.get_arm_type = FRAME;
	the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
	the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
	the_robot->EDP_data.next_interpolation_type= MIM; 
	the_robot->EDP_data.motion_type = ABSOLUTE; 

	the_robot->EDP_data.motion_steps = td.internode_step_no;
	the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++)
	{
		the_robot->EDP_data.next_XYZ_AA_arm_coordinates[i] = 0;
		the_robot->EDP_data.next_force_xyz_torque_xyz[i] = 0;
	}
	
	O_eps_EG_norm=10;
	
	return true;
}

/*
bool ecp_vis_weights_driven_irp6ot::next_step(void){

	return false;
}
*/
