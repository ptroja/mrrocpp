#include <stdio.h>
#include <time.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_local.h"
#include "ecp/conveyor/ecp_g_test.h"

y_simple_generator::y_simple_generator(ecp_task& _ecp_task, int step):
		ecp_generator (_ecp_task)
{
	step_no = step;
}

bool y_simple_generator::first_step ( )
{


	for (int i=0; i<6; i++)
		delta[i]=0.0;



	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;


			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
			the_robot->EDP_data.set_type = ARM_DV;

			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.get_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

	

	return true;
}

bool y_simple_generator::next_step ( )
{
	struct timespec start[9];
	int i; // licznik kolejnych wspolrzednych wektora [0..6]
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
		the_robot->EDP_data.next_motor_arm_coordinates[i] =
		    the_robot->EDP_data.current_motor_arm_coordinates[i]	+ td.coordinate_delta[i];
	}
	
	return true;
}
