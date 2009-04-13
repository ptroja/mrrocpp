#include <stdio.h>
#include <time.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_local.h"
#include "ecp/conveyor/ecp_g_legobrick.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace generator {

incremental_move::incremental_move(common::task::base& _ecp_task, double inc_move):
		base (_ecp_task), move_length(inc_move) {}

bool incremental_move::first_step ( )
{
	td.interpolation_node_no = 1;
	td.internode_step_no = 500;//((int)move_length)*20;
	td.value_in_step_no = td.internode_step_no - 2;


			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
			the_robot->EDP_data.set_type = ARM_DV;

			the_robot->EDP_data.set_arm_type = JOINT;
			the_robot->EDP_data.get_arm_type = JOINT;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.next_interpolation_type = MIM;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

	

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
	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;

	
	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	// (okreslenie kolejnego wezla interpolacji)
	/*if (((node_counter)%100)==0) {
		if( clock_gettime( CLOCK_REALTIME , &start[0]) == -1 ) {
			printf("blad pomiaru czasu");
		}
		printf("ECP conv pomiarow: %d,  czas: %ld, pozycja %f\n", node_counter,
		       start[0].tv_sec%100, the_robot->EDP_data.current_motor_arm_coordinates[0]);
	}*/

	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	td.coordinate_delta[0] = 0.01;//move_length/td.internode_step_no;//((double)(((node_counter%2)*2)-1))/20;   // przyrost wspolrzednej X
	//td.coordinate_delta[1] = 0.0;   // przyrost wspolrzednej Y
	//td.coordinate_delta[2] = 0.0;   // przyrost wspolrzednej Z

	//td.coordinate_delta[3] = 0.0;   // przyrost wspolrzednej FI
	//td.coordinate_delta[4] = 0.0;   // przyrost wspolrzednej TETA
	//td.coordinate_delta[5] = 0.0;   // przyrost wspolrzednej PSI

	//for (int i=0; i<6; i++) {
		// nastepna pozycja
		the_robot->EDP_data.next_joint_arm_coordinates[0] = //td.coordinate_delta[0];
		    the_robot->EDP_data.current_joint_arm_coordinates[0]	+ td.coordinate_delta[0];
	//}
	
	return true;
}

}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp


