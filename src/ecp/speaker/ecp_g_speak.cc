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

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace generator {

speaking_generator::speaking_generator(common::task::ecp_task& _ecp_task, int step):
	 ecp_generator(_ecp_task){	step_no = step;  };

bool speaking_generator::configure(const char* text)
{
	if (text != NULL) {
		strcpy(the_robot->EDP_data.text, text);
		strcpy(the_robot->EDP_data.prosody, "neutral");
		return true;
	} else {
		return false;
	}
}

bool speaking_generator::first_step ( ) {

	for (int i=0; i<6; i++)
		delta[i]=0.0;

	//(sensor_m.begin())->second->base_period=1;
	//(sensor_m.begin())->second->current_period=0;
	communicate_with_edp = true;

	last_sg_state = new_sg_state = SG_FIRST_GET;

	the_robot->EDP_data.instruction_type = GET;

	return true;
}
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



	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	switch (last_sg_state)
	{
		case SG_FIRST_GET:
			if (the_robot->EDP_data.speaking == 0) {
				the_robot->EDP_data.instruction_type = SET;
				new_sg_state = SG_AFTER_SET;
			} else {
				new_sg_state = SG_FIRST_GET;
				usleep(1000 * 20);
			}
			break;
		case SG_AFTER_SET:
			the_robot->EDP_data.instruction_type = GET;
			new_sg_state=SG_LAST_GET;
			break;
		case SG_LAST_GET:
			if (the_robot->EDP_data.speaking == 0) {
				new_sg_state = SG_FINISH;
				communicate_with_edp = false;
			} else {
				new_sg_state = SG_LAST_GET;
				usleep(1000 * 20);
			}
			break;
		case SG_FINISH:
			return false;
			break;
		default:
			break;
	}

//	printf("last %d\n",lastSET);

	// for mic
	// printf("%d \n",(sensor_m.begin())->second->image.sensor_union.mic.word_id);

	// strcpy( the_robot->EDP_data.text, "przesu� kostk� w prawo" );

	return true; //false;
}

}
} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

