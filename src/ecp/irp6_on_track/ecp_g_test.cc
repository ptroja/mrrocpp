// -------------------------------------------------------------------------
//                            ecp_gen_test.cc
//            Effector Control Process (ECP) - force & torque methods
// 			Funkcje do tworzenia procesow ECP z wykorzystaniem sily
// 			By Slawek Bazant & Tomasz Winiarski
// 			Ostatnia modyfikacja: 06.10.2005r.
// -------------------------------------------------------------------------

#include <stdio.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/irp6_on_track/ecp_local.h"
#include "ecp/irp6_on_track/ecp_g_test.h"
#include "lib/mathtr.h"


y_simple_generator::y_simple_generator(ecp_task& _ecp_task, int step):
	 ecp_generator (_ecp_task, true) { 		step_no = step;          	};  


bool y_simple_generator::first_step ( ) {

	run_counter = 0;
	second_step = false;
	for (int j=0; j<3; j++)
		for (int i=0; i<4; i++)
			previous_frame[i][j]=0;

	for (int i=0; i<6; i++)
		delta[i]=0.0;


	
	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;


		the_robot->EDP_data.instruction_type = SET_GET;
		the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
		the_robot->EDP_data.set_type = RMODEL_DV;
		the_robot->EDP_data.set_arm_type = FRAME;
		the_robot->EDP_data.set_rmodel_type = TOOL_FRAME;
		for(int i=0; i<4; i++)
			for(int j=0; j<3; j++)
				if(i==j) the_robot->EDP_data.next_tool_frame[i][j]=1;
//				else if (i==3 && j==1) the_robot->EDP_data.next_tool_frame[i][j]=-0.1;
//				else if (i==3 && j==2) the_robot->EDP_data.next_tool_frame[i][j]= 0.08;
				else the_robot->EDP_data.next_tool_frame[i][j]=0;
		the_robot->EDP_data.get_arm_type = FRAME;
		the_robot->EDP_data.motion_type = ABSOLUTE;
		the_robot->EDP_data.motion_steps = td.internode_step_no;
		the_robot->EDP_data.value_in_step_no = td.value_in_step_no;
	

	return true;
}; // end: bool y_simple_generator::first_step (std::map <SENSOR_ENUM, sensor*>& sensor_m, robot& the_robot )
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool y_simple_generator::next_step (std::map <SENSOR_ENUM, sensor*>& sensor_m ) {
	struct timespec start[9];
	
	
//	printf("bbb\n");
	if (ecp_t->pulse_check()) 
	{
		return false;
	}
	

	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.set_type = ARM_DV;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;
	

	double axis_table[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	vector x_axis(axis_table[0]);
	vector y_axis(axis_table[1]);
	vector z_axis(axis_table[2]);	
	Homog_matrix curr_frame(the_robot->EDP_data.current_arm_frame);								// pobranie aktualnej ramki (tylko na poczatku ruchu)
	Homog_matrix prev_frame(previous_frame);
	Homog_matrix next_frame;	
	Homog_matrix temp = !curr_frame;		
	Homog_matrix temp2 = !prev_frame;																	// odwrocenie aktualnej ramki do mnozenia przez ramke obrotu
	temp2.move(0,0,0);																							// oczysczenie z wszelkiego ruchu
	temp.move(0,0,0);
	const double FORCE_TO_MOVE_RATIO = 0.00005;
	const double TORQUE_TO_ROTATE_RATIO = 0.0008;
	double move_tab[3] = {0,0,0};
	bool force_mov[3] = {false,false,false};
	double mov[3] = { move_tab[0] + force_mov[0]*FORCE_TO_MOVE_RATIO*(sensor_m.begin())->second->image.force.rez[0] , 		// tablica zmiany pozycji wzgledem sil
							move_tab[1] + force_mov[1]*FORCE_TO_MOVE_RATIO*(sensor_m.begin())->second->image.force.rez[1] , 
							move_tab[2] + force_mov[2]*FORCE_TO_MOVE_RATIO*(sensor_m.begin())->second->image.force.rez[2]};
	vector move_vector(mov);
	Homog_matrix temporary_frame;
	if(!second_step) temporary_frame = curr_frame;
	else temporary_frame = prev_frame;
	temporary_frame.move(0,0,0);
	move_vector =  temporary_frame * move_vector;
	move_vector.to_table(mov);
	double rot_tab[3] = {0,0,0/*.0001*/};
	bool force_rot[3] = {false,false,false};
	double rot[3] = {	rot_tab[0] + force_rot[0]*TORQUE_TO_ROTATE_RATIO*(sensor_m.begin())->second->image.force.rez[3] , 		// tablica zmiany orientacji wzgledem momentow sill
							rot_tab[1] + force_rot[1]*TORQUE_TO_ROTATE_RATIO*(sensor_m.begin())->second->image.force.rez[4] , 
							rot_tab[2] + force_rot[2]*TORQUE_TO_ROTATE_RATIO*(sensor_m.begin())->second->image.force.rez[5]};																			
	Homog_matrix move_frame(mov[0], mov[1], mov[2]);
	
	Homog_matrix rot_frame(x_axis, y_axis, z_axis, rot);											
	if(!second_step)
	{
		next_frame = move_frame * curr_frame * rot_frame;											// wyliczenie nowej ramki
		second_step = true;
	}
	else next_frame = move_frame * prev_frame * rot_frame;
	next_frame.to_table(previous_frame);
	copy_frame(the_robot->EDP_data.next_arm_frame, previous_frame);
						// przepisanie nowej ramki do EDP
	the_robot->EDP_data.next_gripper_coordinate=the_robot->EDP_data.current_gripper_coordinate;
	
// // 	if(++run_counter==1000) return false;	
	return true;
};  // end:  y_simple_generator::next_step (, robot& the_robot )
