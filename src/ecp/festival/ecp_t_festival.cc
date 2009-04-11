#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ecp/festival/ecp_g_festival.h"
#include "ecp_mp/ecp_mp_t_festival.h"
#include "ecp/festival/ecp_t_festival.h"

namespace mrrocpp {
namespace ecp {
namespace common {

// KONSTRUKTORY
ecp_task_festival::ecp_task_festival(configurator &_config) :
	ecp_task(_config)
{
	fg = new festival_generator (*this);
}

ecp_task_festival::~ecp_task_festival()
{
	delete fg;
}

// methods for ECP template to redefine in concrete classes
void ecp_task_festival::task_initialization(void)
{
	sr_ecp_msg->message("ECP loaded");
}

void ecp_task_festival::main_task_algorithm(void)
{
	int isTest = config.return_int_value("test_mode");

	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("NEXT_STATE received");

		switch ( (ecp_mp::task::ECP_FESTIVAL_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
			case ecp_mp::task::ECP_GEN_FESTIVAL:
				if(isTest)
					sr_ecp_msg->message(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				else
				{
					fg->set_voice((festival_generator::VOICE) mp_command.ecp_next_state.mp_2_ecp_next_state_variant);
					fg->set_phrase(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
					fg->Move();
				}
				break;
			default:
				fprintf(stderr, "invalid ecp_next_state.mp_2_ecp_next_state (%d)\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
				break;
		}

		ecp_termination_notice();
	}
}

ecp_task* return_created_ecp_task(configurator &_config)
{
	return new ecp_task_festival(_config);
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp

