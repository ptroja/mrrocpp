#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/player/ecp_g_playerpos.h"
#include "ecp/player/ecp_t_playerpos.h"
#include "ecp_mp/ecp_mp_t_player.h"

// KONSTRUKTORY
ecp_task_playerpos::ecp_task_playerpos(configurator &_config) :
	ecp_task(_config)
{
	ppg = new playerpos_generator (*this);
}

ecp_task_playerpos::~ecp_task_playerpos()
{
	delete ppg;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_playerpos::task_initialization(void)
{
	sr_ecp_msg->message("ECP loaded");
}

void ecp_task_playerpos::main_task_algorithm(void)
{
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");

		switch ( (ECP_PLAYER_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
			case ECP_GEN_PLAYERPOS:
				ppg->set_goal(mp_command.ecp_next_state.playerpos_goal);
				ppg->Move();
				break;
			default:
				fprintf(stderr, "invalid ecp_next_state.mp_2_ecp_next_state (%d)\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
				break;
		}

		ecp_termination_notice();
	}
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_playerpos(_config);
}
