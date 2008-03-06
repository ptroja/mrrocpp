// -------------------------------------------------------------------------
//                             ecp_gen_test.cc
//             Effector Control Process (ECP) - force & torque methods
// 			Test nowego EDP z wykorzystaniem sily
// 			By Slawek Bazant
//			Ostatnia modyfikacja: 05.01.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_postument/ecp_local.h"
#include "ecp/irp6_postument/ecp_g_test.h"

#include "lib/mathtr.h"

y_simple_generator::y_simple_generator(ecp_task& _ecp_task, int step):
	 ecp_generator (_ecp_task, true){ 	step_no = step;	};

bool y_simple_generator::first_step ( ) {

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);


		the_robot->EDP_data.instruction_type = GET;
		the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
		the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->EDP_data.set_type = ARM_DV;
		the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
		the_robot->EDP_data.get_rmodel_type = TOOL_FRAME;
		the_robot->EDP_data.set_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
		the_robot->EDP_data.get_arm_type = POSE_FORCE_TORQUE_AT_FRAME;
		the_robot->EDP_data.motion_type = RELATIVE;
		the_robot->EDP_data.motion_steps = td.internode_step_no;
		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
		
		

		for(int i=0;i<6;i++) {
			the_robot->EDP_data.next_position_velocity[i] = 0;
			the_robot->EDP_data.next_force_xyz_torque_xyz[i] = 0;	
		//	the_robot->EDP_data.selection_vector[i] = FORCE_SV_AX;
		}
		
		the_robot->EDP_data.next_force_xyz_torque_xyz[2] = -5;
		
		for (int i=0;i<3;i++)
		{
			the_robot->EDP_data.next_inertia[i] = FORCE_INERTIA;
			the_robot->EDP_data.next_inertia[i+3] = TORQUE_INERTIA;
			the_robot->EDP_data.next_reciprocal_damping[i] = FORCE_RECIPROCAL_DAMPING;
			the_robot->EDP_data.next_reciprocal_damping[i+3] = TORQUE_RECIPROCAL_DAMPING;
		}
		
		//	the_robot->EDP_data.selection_vector[0] = POSE_SV_AX;
		//	the_robot->EDP_data.selection_vector[1] = POSE_SV_AX;
		//	the_robot->EDP_data.selection_vector[2] = POSE_SV_AX;
		//	the_robot->EDP_data.selection_vector[3] = POSE_SV_AX;
		//	the_robot->EDP_data.selection_vector[4] = POSE_SV_AX;
		//	the_robot->EDP_data.selection_vector[5] = POSE_SV_AX;
		//	the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[0] = -4;
		//	the_robot->EDP_data.ECPtoEDP_pos_xyz_rot_xyz[1] = 0.0001;
		//	for (int i=2;i<6;i++) the_robot->EDP_data.selection_vector[i] = POSE_SV_AX;
		//	the_robot->EDP_data.ECPtoEDP_force_xyz_torque_xyz[5] = 1;


	

	return true;
}; // end: bool y_simple_generator::first_step (map <SENSOR_ENUM, sensor*>& sensor_m, robot& the_robot )
//--------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_simple_generator::next_step ( ) {
	// static int count;
	// struct timespec start[9];
	if (ecp_t.pulse_check()) {
		ecp_t.mp_buffer_receive_and_send ();
		return false;
	} 
	the_robot->EDP_data.instruction_type = SET_GET;


	/*if(the_robot->EDP_data.current_gripper_coordinate < 0.058) */
	the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;
/*
 	double wx = sensor_m.begin()->second->image.force.rez[0];
 	double wy = sensor_m.begin()->second->image.force.rez[1];
 	
 	double v = sqrt (wx*wx + wy*wy);

 	double s_alfa = wy / v;
 	double c_alfa = - wx / v;
 
	 the_robot->EDP_data.ECPtoEDP_pos_xyz_rot_xyz[1] = -0.00006*v;
 
 	the_robot->EDP_data.ECPtoEDP_reference_frame[0][0] = c_alfa;
	the_robot->EDP_data.ECPtoEDP_reference_frame[0][1] = s_alfa;
	
	the_robot->EDP_data.ECPtoEDP_reference_frame[1][0] = -s_alfa;
	the_robot->EDP_data.ECPtoEDP_reference_frame[1][1] = c_alfa;

 	the_robot->EDP_data.ECPtoEDP_reference_frame[0][0] = 1;
	the_robot->EDP_data.ECPtoEDP_reference_frame[0][1] = 0;
	
	the_robot->EDP_data.ECPtoEDP_reference_frame[1][0] = 0;
	the_robot->EDP_data.ECPtoEDP_reference_frame[1][1] = 1; 

printf("sensor: x: %+d, y: %+d, v:%+d\n", (int)round(wx),  (int)round(wy), (int)round(v));
*/

printf("sensor: z: %f\n", the_robot->EDP_data.current_force_xyz_torque_xyz[2]);


//	else the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate-0.0001;
	
/*	
	frame_tab beggining_frame;
	copy_frame(beggining_frame,the_robot->EDP_data.current_beggining_arm_frame);
	Homog_matrix beg_frame = Homog_matrix(beggining_frame);
	cout << endl << "ecp: beginning_frame" << endl << endl << beg_frame;
	frame_tab present_frame;
	copy_frame(present_frame,the_robot->EDP_data.current_present_arm_frame);
	Homog_matrix pres_frame = Homog_matrix(present_frame);
	cout << endl << "ecp: present_frame" << endl << endl << pres_frame;
	frame_tab predicted_frame;
	copy_frame(predicted_frame,the_robot->EDP_data.current_predicted_arm_frame);
	Homog_matrix pred_frame = Homog_matrix(predicted_frame);
	cout << endl << "ecp: predicted_frame" << endl << endl<< pred_frame;
	double force[6];
	for (int i=0;i<6;i++) force[i] = the_robot->EDP_data.current_force_xyz_torque_xyz[i];
	cout << "force" << endl << endl;
	for(int i=0;i<6;i++) cout << force[i] << "  " ;
	cout << endl;
*/
//	if(++run_counter==1000) return false;	
	return true;
}; // end:  y_simple_generator::next_step (, robot& the_robot )
