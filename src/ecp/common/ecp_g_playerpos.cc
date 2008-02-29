#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/common/ecp_g_playerpos.h"
#include "ecp_mp/ecp_mp_tr_player.h"

playerpos_generator::playerpos_generator(ecp_task& _ecp_task):
	ecp_generator (_ecp_task, true)
{
	pc = new PlayerClient("192.168.1.64", 6665);
	assert(pc);
	pp = new PositionProxy(pc, 1, 'a');
	assert(pp);
}

bool playerpos_generator::first_step ( ) 
{
	ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);

	ecp_t.mp_buffer_receive_and_send ();

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
		ecp_t.mp_buffer_receive_and_send ();
		return false;
	}
	else
	{
		ecp_t.set_ecp_reply (ECP_ACKNOWLEDGE);
		ecp_t.mp_buffer_receive_and_send ();
	}

#if 0
    if(pc->Peek(0)) {
    	pc->Read();
    }
#else
   	pc->Read();
#endif

    if (pp->fresh) {
    	printf("odometry: [%f, %f %f]\n", pp->xpos, pp->ypos, pp->theta);
    	pp->fresh = false;
    } else {
    	printf("no new data\n");
    }
    
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
