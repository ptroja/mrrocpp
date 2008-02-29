#include <stdio.h>
#include <time.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/conveyor/ecp_local.h"
#include "ecp/conveyor/ecp_g_test.h"

y_simple_generator::y_simple_generator(ecp_task& _ecp_task, int step):
		ecp_generator (_ecp_task, true)
{
	step_no = step;
}

bool y_simple_generator::first_step ( )
{

	ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
	for (int i=0; i<6; i++)
		delta[i]=0.0;

	ecp_t.mp_buffer_receive_and_send ();
	node_counter = 0;

	td.interpolation_node_no = 1;
	td.internode_step_no = step_no;
	td.value_in_step_no = td.internode_step_no - 2;

	switch ( ecp_t.mp_command_type() ) {
		case NEXT_POSE:
			the_robot->EDP_data.instruction_type = GET;
			the_robot->EDP_data.get_type = ARM_DV; // arm - ORYGINAL
			the_robot->EDP_data.set_type = ARM_DV;

			the_robot->EDP_data.set_arm_type = MOTOR;
			the_robot->EDP_data.get_arm_type = MOTOR;
			the_robot->EDP_data.motion_type = ABSOLUTE;
			the_robot->EDP_data.motion_steps = td.internode_step_no;
			the_robot->EDP_data.value_in_step_no = td.value_in_step_no;

			the_robot->create_command ();
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			printf("post first step in mp comm: %d\n", ecp_t.mp_command_type());
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	} // end: switch

	return true;
}

bool y_simple_generator::next_step ( )
{
	struct timespec start[9];
	int i; // licznik kolejnych wspolrzednych wektora [0..6]
	if (ecp_t.pulse_check()) { // Koniec odcinka
		ecp_t.mp_buffer_receive_and_send ();
		return false;
	} else { // w trakcie interpolacji
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.mp_buffer_receive_and_send ();
	}
	// Kopiowanie danych z bufora przyslanego z EDP do
	// obrazu danych wykorzystywanych przez generator
	the_robot->get_reply();
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	// nie aktualizujemy pozycjio na podstawie odczytu z EDP
	the_robot->EDP_data.instruction_type = SET;
	the_robot->EDP_data.get_type = NOTHING_DV;
	the_robot->EDP_data.get_arm_type = INVALID_END_EFFECTOR;

	node_counter++;
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
	switch ( ecp_t.mp_command_type() ) {
		case NEXT_POSE:
			the_robot->create_command ();
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			printf("post next step in mp comm: %d\n", ecp_t.mp_command_type());
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}
	return true;
}
