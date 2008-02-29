// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (ECP) - speaker generators
// 
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/speaker/ecp_local.h"
#include "ecp/speaker/ecp_g_speak.h"

speaking_generator::speaking_generator(ecp_task& _ecp_task, int step):
	 ecp_generator(_ecp_task, true){	step_no = step;  };  

bool speaking_generator::configure(char* text)
{

	if (text!=NULL)
	{ 
		strcpy( the_robot->EDP_data.text, text);
		strcpy( the_robot->EDP_data.prosody, "neutral");
		return true;
	} else
	{
		return false;
	}
}


bool speaking_generator::first_step ( ) {

	ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
	for (int i=0; i<6; i++)
		delta[i]=0.0;

	//(sensor_m.begin())->second->base_period=1;
 	//(sensor_m.begin())->second->current_period=0;
	the_robot->communicate = true;
	ecp_t.mp_buffer_receive_and_send ();
	node_counter = 0;

	last_sg_state = new_sg_state = SG_FIRST_GET;

	switch ( ecp_t.mp_command_type() ) {
	case NEXT_POSE:
		the_robot->EDP_data.instruction_type = GET;

		the_robot->create_command ();
	break;
	case STOP:
		throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
	case END_MOTION:
		return false;
	break;
	case INVALID_COMMAND:
	default:
		throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	} // end: switch

	return true;
}; // end: bool y_nose_run_force_generator::first_step ( )
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool speaking_generator::next_step ( ) {
	// struct timespec start[9];

/*
	if (pulse_check(the_robot->trigger_attach)) { // Koniec odcinka
		ecp_t.mp_buffer_receive_and_send ();
		return false;
	} else { // w trakcie interpolacji
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.mp_buffer_receive_and_send ();
	}
	*/ //odrem jako niezalezny od rcsc generator 
	
	last_sg_state = new_sg_state;
	
	if (last_sg_state != SG_FINISH)
	{
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.mp_buffer_receive_and_send ();
	}
	else
	{
		ecp_t.ecp_termination_notice();
	}
	
	
	// Kopiowanie danych z bufora przyslanego z EDP do
	// obrazu danych wykorzystywanych przez generator
	the_robot->get_reply();
	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	switch (last_sg_state)
	{
		case SG_FIRST_GET:
			if (the_robot->EDP_data.speaking==0)
			{
				the_robot->EDP_data.instruction_type = SET;
				new_sg_state=SG_AFTER_SET;
			}
			else
			{
				new_sg_state=SG_FIRST_GET;
				usleep(1000*20);
			}
		break;
		case SG_AFTER_SET:
			the_robot->EDP_data.instruction_type = GET;
			new_sg_state=SG_LAST_GET;
		break;
		case SG_LAST_GET:
			if (the_robot->EDP_data.speaking==0)
			{
				new_sg_state=SG_FINISH;	
				the_robot->communicate = false;
			}
			else
			{
				new_sg_state=SG_LAST_GET;
				usleep(1000*20);
			}
		break;
		case SG_FINISH:
		break;
		default:
		break;
	}
	
	
//	printf("last %d\n",lastSET);
	node_counter++;
	
	// for mic
	// printf("%d \n",(sensor_m.begin())->second->image.mic.word_id);
	
	// strcpy( the_robot->EDP_data.text, "przesuñ kostkê w prawo" );
	 

	switch ( ecp_t.mp_command_type() ) {
	case NEXT_POSE:
		if (last_sg_state != SG_FINISH)
		{
			the_robot->create_command ();
		}
		else
		{
			return false;
		}
	break;
	case STOP:
		throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
	case END_MOTION:
		return false;
	break;
	case INVALID_COMMAND:
	default:
		throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	} // // end: switch
	return true; //false;
}; //  end:  y_nose_run_force_generator::next_step ( )

