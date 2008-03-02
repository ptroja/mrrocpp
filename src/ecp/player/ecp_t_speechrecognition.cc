#include <stdio.h>
#include <string.h>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "ecp/player/ecp_g_speechrecognition.h"
#include "ecp/player/ecp_t_speechrecognition.h"
#include "ecp_mp/ecp_mp_t_player.h"

// KONSTRUKTORY
ecp_task_speechrecognition::ecp_task_speechrecognition(configurator &_config)
	: ecp_task(_config)
{
	srg = new speechrecognition_generator (*this);
}

ecp_task_speechrecognition::~ecp_task_speechrecognition()
{
	delete srg;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_speechrecognition::task_initialization(void)
{
	sr_ecp_msg->message("ECP loaded");
}

void ecp_task_speechrecognition::main_task_algorithm(void)
{
	sr_ecp_msg->message("ECP speechrecognition - wcisnij start");
	ecp_wait_for_start();

	for(;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state ();

		sr_ecp_msg->message("Order received");

		switch ( (ECP_PLAYER_STATES) mp_command.mp_package.mp_2_ecp_next_state) {
			case ECP_GEN_SPEECHRECOGNITION:
				Move (*srg);
				break;
			default:
				fprintf(stderr, "invalid mp_2_ecp_next_state (%d)\n", mp_command.mp_package.mp_2_ecp_next_state);
				break;
		}

		ecp_termination_notice();
	}

	// Oczekiwanie na STOP
	printf("przed wait for stop\n");
	ecp_wait_for_stop ();
}

ecp_task* return_created_ecp_task (configurator &_config)
{
	return new ecp_task_speechrecognition(_config);
}