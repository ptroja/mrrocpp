#include <stdio.h>
#include <math.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ecp/common/ecp_g_playerpos.h"
#include "ecp_mp/ecp_mp_tr_player.h"

playerpos_generator::playerpos_generator(ecp_task& _ecp_task):
	ecp_generator (_ecp_task, true)
{	
}

bool playerpos_generator::first_step ( ) 
{
	ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);

	ecp_t.get_mp_command ();

	switch ( ecp_t.mp_command_type() ) 
	{
		case NEXT_POSE:
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	return true;
}

bool playerpos_generator::next_step ( ) 
{
	if (ecp_t.pulse_check()) 
	{
		ecp_t.get_mp_command ();
		return false;
	}
	else
	{
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.get_mp_command ();
	}

    transmitter_m[TRANSMITTER_PLAYER]->t_read(1);
    printf("odometry: [%f, %f %f]\n",
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_position.px,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_position.py,
            transmitter_m[TRANSMITTER_PLAYER]->from_va.player_position.pa
          );
	
	switch ( ecp_t.mp_command_type() ) 
	{
		case NEXT_POSE:
			//the_robot->create_command ();
			break;
		case STOP:
			throw ECP_error (NON_FATAL_ERROR, ECP_STOP_ACCEPTED);
		case END_MOTION:
		case INVALID_COMMAND:
		default:
			throw ECP_error(NON_FATAL_ERROR, INVALID_MP_COMMAND);
	}

	return true;
}
