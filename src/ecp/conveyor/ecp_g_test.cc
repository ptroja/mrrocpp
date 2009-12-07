#include <stdio.h>
#include <time.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_r_conv.h"
#include "ecp/conveyor/ecp_g_test.h"

namespace mrrocpp {
namespace ecp {
namespace conveyor {
namespace generator {

y_simple::y_simple(common::task::task& _ecp_task, int step):
	generator (_ecp_task)
{
	step_no = step;
}

bool y_simple::first_step ( )
{


	for (int i=0; i<6; i++)
		delta[i]=0.0;



	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;


			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			the_robot->ecp_command.instruction.get_type = ARM_DV; // arm - ORYGINAL
			the_robot->ecp_command.instruction.set_type = ARM_DV;

			the_robot->ecp_command.instruction.set_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.get_arm_type = lib::MOTOR;
			the_robot->ecp_command.instruction.motion_type = lib::ABSOLUTE;
			 the_robot->ecp_command.instruction.interpolation_type = lib::MIM;
			the_robot->ecp_command.instruction.motion_steps = td.internode_step_no;
			the_robot->ecp_command.instruction.value_in_step_no = td.value_in_step_no;

	

	return true;
}

bool y_simple::next_step ( )
{
	struct timespec start[9];
	int i; // licznik kolejnych wspolrzednych wektora [0..6]
	if (check_and_null_trigger()) { // Koniec odcinka
		return false;
	} 
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	// nie aktualizujemy pozycjio na podstawie odczytu z EDP
	the_robot->ecp_command.instruction.instruction_type = lib::SET;
	the_robot->ecp_command.instruction.get_type = NOTHING_DV;
	the_robot->ecp_command.instruction.get_arm_type = lib::INVALID_END_EFFECTOR;

	
	// Obliczenie zadanej pozycji posredniej w tym kroku ruchu
	// (okreslenie kolejnego wezla interpolacji)
	if (((node_counter)%100)==0) {
		if( clock_gettime( CLOCK_REALTIME , &start[0]) == -1 ) {
			printf("blad pomiaru czasu");
		}
		printf("ECP conv pomiarow: %d,  czas: %ld, pozycja %f\n", node_counter,
		       start[0].tv_sec%100, the_robot->EDP_data.current_motor_arm_coordinates[0]);
	}

	// Wspolrzedne kartezjanskie XYZ i katy Eulera ZYZ
	td.coordinate_delta[0] = ((double)(((node_counter%2)*2)-1))/20;   // przyrost wspolrzednej X
	// 	td.coordinate_delta[0] = 1;   // przyrost wspolrzednej Y
	td.coordinate_delta[1] = 0.0;   // przyrost wspolrzednej Y
	td.coordinate_delta[2] = 0.0;   // przyrost wspolrzednej Z

	td.coordinate_delta[3] = 0.0;   // przyrost wspolrzednej FI
	td.coordinate_delta[4] = 0.0;   // przyrost wspolrzednej TETA
	td.coordinate_delta[5] = 0.0;   // przyrost wspolrzednej PSI

	for (i=0; i<6; i++) {
		// nastepna pozycja
		the_robot->ecp_command.instruction.arm.pf_def.arm_coordinates[i] =
		    the_robot->EDP_data.current_motor_arm_coordinates[i]	+ td.coordinate_delta[i];
	}
	
	return true;
}

}
} // namespace conveyor
} // namespace ecp
} // namespace mrrocpp


