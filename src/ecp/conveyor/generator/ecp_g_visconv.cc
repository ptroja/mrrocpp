#include <stdio.h>
#include <time.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_r_conv.h"
//#include "ecp/conveyor/generator/ecp_g_legobrick.h"
#include "ecp/conveyor/generator/ecp_g_visconv.h"
#include <iostream>
#include "math.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace generator {

incremental_move::incremental_move(common::task::task& _ecp_task, double inc_move):
	generator (_ecp_task), move_length(inc_move) {}

bool incremental_move::first_step ( )
{
	td.interpolation_node_no = 1;
	td.internode_step_no = 50;//((int)move_length)*20;
	td.value_in_step_no = td.internode_step_no - 2;


			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
			the_robot->ecp_command.instruction.set_type = ARM_DV;

			the_robot->ecp_command.instruction.set_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.get_arm_type = lib::JOINT;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

			first=1;
			stepno=ecp_t.config.return_int_value("steps");//100;

	return true;
}

bool incremental_move::next_step ( )
{
	//struct timespec start[9];
	if (check_and_null_trigger()) { // Koniec odcinka
		return false;
	} 
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	// nie aktualizujemy pozycjio na podstawie odczytu z EDP
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.get_type = NOTHING_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
	
	if(first)
	{
		current_pose=the_robot->reply_package.arm.pf_def.arm_coordinates[0];
		begin_pose=current_pose;
		step=1;
		first=0;
	}
	
	
	//std::cout<<"ABS " << current_pose << std::endl; 
	
	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	// (okreslenie kolejnego wezla interpolacji)
	/*if (((node_counter)%100)==0) {
		if( clock_gettime( CLOCK_REALTIME , &start[0]) == -1 ) {
			printf("blad pomiaru czasu");
		}
		printf("ECP conv pomiarow: %d,  czas: %ld, pozycja %f\n", node_counter,
		       start[0].tv_sec%100, the_robot->reply_package.arm.pf_def.arm_coordinates[0]);
	}*/

	
	//td.coordinate_delta[0] = -0.01; //0.01//move_length/td.internode_step_no;//((double)(((node_counter%2)*2)-1))/20;   // przyrost wspolrzednej X


	//next_pose = current_pose -0.005;
	next_pose=begin_pose-0.1*(1-cos(3.14*2*step/((double)(stepno))));
	//next_pose=0.005*cos(3.14*0.01*step);
	//std::cout << "BGN: " << begin_pose << "ABS: " << next_pose << std::endl;
	
	gettimeofday(&acctime,NULL);
	std::cout << acctime.tv_sec << " " << acctime.tv_usec << " " << next_pose << " ";
	std::cout << std::endl;
	
	the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[0] = next_pose;//the_robot->reply_package.arm.pf_def.arm_coordinates[0]; //next_pose;
	//current_pose=next_pose;	
	
	step++;
	step%=stepno;
	
	//   the_robot->reply_package.arm.pf_def.arm_coordinates[0]	+ td.coordinate_delta[0];
	//}
	
	return true;
}

}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp


