// -------------------------------------------------------------------------
//                             ecp_gen_test.cc
//             Effector Control Process (lib::ECP) - force & torque methods
// 			Test nowego EDP z wykorzystaniem sily
// 			By Slawek Bazant
//			Ostatnia modyfikacja: 05.01.2006r.
// -------------------------------------------------------------------------

#include <stdio.h>
#include <iostream>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_postument/ecp_r_irp6p.h"
#include "ecp/irp6_postument/ecp_g_test.h"

#include "lib/mathtr.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace generator {

y_simple::y_simple(common::task::task& _ecp_task, int step) :
	generator(_ecp_task)
{
	step_no = step;
}
;

bool y_simple::first_step()
{

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	lib::Homog_matrix tool_frame(0.0, 0.0, 0.25);
	tool_frame.get_frame_tab(the_robot->EDP_data.next_tool_frame);

	the_robot->EDP_data.instruction_type = lib::GET;
	the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
	the_robot->EDP_data.set_type = ARM_DV | RMODEL_DV;
	//	the_robot->EDP_data.set_type = ARM_DV;
	the_robot->EDP_data.set_rmodel_type = lib::TOOL_FRAME;
	the_robot->EDP_data.get_rmodel_type = lib::TOOL_FRAME;
	the_robot->EDP_data.set_arm_type = lib::PF_VELOCITY;
	the_robot->EDP_data.get_arm_type = lib::FRAME;
	the_robot->EDP_data.motion_type = lib::RELATIVE;
	the_robot->EDP_data.next_interpolation_type = lib::TCIM;
	the_robot->EDP_data.motion_steps = td.internode_step_no;
	the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

	for (int i=0; i<6; i++) {
		the_robot->EDP_data.next_velocity[i] = 0;
		the_robot->EDP_data.next_force_xyz_torque_xyz[i] = 0;
		//	the_robot->EDP_data.selection_vector[i] = FORCE_SV_AX;
	}

	the_robot->EDP_data.next_force_xyz_torque_xyz[2] = -5;

	for (int i=0; i<3; i++) {
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
}
; // end: bool y_simple_generator::first_step (map <lib::SENSOR_ENUM, lib::sensor*>& sensor_m, robot& the_robot )
//--------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_simple::next_step()
{
	// static int count;
	// struct timespec start[9];
	if (check_and_null_trigger()) {
		ecp_t.mp_buffer_receive_and_send();
		return false;
	}
	the_robot->EDP_data.instruction_type = lib::SET_GET;

	/*if(the_robot->EDP_data.current_gripper_coordinate < 0.058) */
	the_robot->EDP_data.next_gripper_coordinate = the_robot->EDP_data.current_gripper_coordinate;
	/*
	 double wx = sensor_m.begin()->second->image.sensor_union.force.rez[0];
	 double wy = sensor_m.begin()->second->image.sensor_union.force.rez[1];
	 
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
	 lib::frame_tab beggining_frame;
	 lib::copy_frame(beggining_frame,the_robot->EDP_data.current_arm_frame);
	 lib::Homog_matrix beg_frame = lib::Homog_matrix(beggining_frame);
	 cout << endl << "ecp: beginning_frame" << endl << endl << beg_frame;
	 lib::frame_tab present_frame;
	 lib::copy_frame(present_frame,the_robot->EDP_data.current_arm_frame);
	 lib::Homog_matrix pres_frame = lib::Homog_matrix(present_frame);
	 cout << endl << "ecp: present_frame" << endl << endl << pres_frame;
	 lib::frame_tab predicted_frame;
	 lib::copy_frame(predicted_frame,the_robot->EDP_data.current_arm_frame);
	 lib::Homog_matrix pred_frame = lib::Homog_matrix(predicted_frame);
	 cout << endl << "ecp: predicted_frame" << endl << endl<< pred_frame;
	 double force[6];
	 for (int i=0;i<6;i++) force[i] = the_robot->EDP_data.current_force_xyz_torque_xyz[i];
	 cout << "force" << endl << endl;
	 for(int i=0;i<6;i++) cout << force[i] << "  " ;
	 cout << endl;
	 */
	//	if(++run_counter==1000) return false;	
	return true;
}
; // end:  y_simple_generator::next_step (, robot& the_robot )

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp
