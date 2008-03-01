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
	

	return true;
}

bool playerpos_generator::next_step ( ) 
{
	if (ecp_t.pulse_check()) 
	{
		ecp_t.mp_buffer_receive_and_send ();
		return false;
	}
	
	
#if 0
	// do not block
    if(pc->Peek(0)) {
    	pc->Read();
    }
#else
    // block
   	pc->Read();
#endif

    if (pp->fresh) {
    	printf("odometry: [%f, %f %f]\n", pp->xpos, pp->ypos, pp->theta);
    	pp->fresh = false;
    } else {
    	printf("no new data\n");
    }

	return true;
}
