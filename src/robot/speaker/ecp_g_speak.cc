// -------------------------------------------------------------------------
//                            ecp.cc
//            Effector Control Process (lib::ECP) - speaker generators
//
// Ostatnia modyfikacja: 2006
// -------------------------------------------------------------------------

#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "robot/speaker/ecp_r_speaker.h"
#include "robot/speaker/ecp_g_speak.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace generator {

speaking::speaking(common::task::task& _ecp_task, int step) :
	generator(_ecp_task)
{
	step_no = step;
}
;

bool speaking::configure(const char* text)
{
	if (text != NULL) {
		strcpy(the_robot->ecp_command.instruction.arm.text_def.text, text);
		strcpy(the_robot->ecp_command.instruction.arm.text_def.prosody, "neutral");
		return true;
	} else {
		return false;
	}
}

bool speaking::first_step()
{

	for (int i = 0; i < 6; i++)
		delta[i] = 0.0;

	//(sensor_m.begin())->second->base_period=1;
	//(sensor_m.begin())->second->current_period=0;
	if (the_robot)
		the_robot->communicate_with_edp = true;

	last_sg_state = new_sg_state = SG_FIRST_GET;

	the_robot->ecp_command.instruction.instruction_type = lib::GET;

	return true;
}
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
bool speaking::next_step()
{
	// struct timespec start[9];

	/*
	 if (pulse_check(the_robot->trigger_attach)) { // Koniec odcinka
	 ecp_t.mp_buffer_receive_and_send ();
	 return false;
	 } else { // w trakcie interpolacji
	 ecp_t.set_ecp_reply (lib::ECP_ACKNOWLEDGE);
	 ecp_t.mp_buffer_receive_and_send ();
	 }
	 *///odrem jako niezalezny od rcsc generator

	last_sg_state = new_sg_state;

	// Przygotowanie kroku ruchu - do kolejnego wezla interpolacji

	switch (last_sg_state)
	{
		case SG_FIRST_GET:
			if (the_robot->reply_package.arm.text_def.speaking == 0) {
				the_robot->ecp_command.instruction.instruction_type = lib::SET;
				new_sg_state = SG_AFTER_SET;
			} else {
				new_sg_state = SG_FIRST_GET;
				usleep(1000 * 20);
			}
			break;
		case SG_AFTER_SET:
			the_robot->ecp_command.instruction.instruction_type = lib::GET;
			new_sg_state = SG_LAST_GET;
			break;
		case SG_LAST_GET:
			if (the_robot->reply_package.arm.text_def.speaking == 0) {
				new_sg_state = SG_FINISH;
				if (the_robot)
					the_robot->communicate_with_edp = false;
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

	// strcpy( the_robot->ecp_command.instruction.arm.text_def.text, "przesu� kostk� w prawo" );

	return true; //false;
}

}
} // namespace speaker
} // namespace ecp
} // namespace mrrocpp

