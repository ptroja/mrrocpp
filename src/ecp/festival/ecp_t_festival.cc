#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "ecp/festival/ecp_g_festival.h"
#include "ecp_mp/ecp_mp_t_festival.h"
#include "ecp/festival/ecp_t_festival.h"

namespace mrrocpp {
namespace ecp {
namespace festival {
namespace task {

// KONSTRUKTORY
task::task(lib::configurator &_config) :
	common::task::task(_config)
{
	fg = new generator::base (*this);
}

task::~task()
{
	delete fg;
}

// methods for ECP template to redefine in concrete classes
void task::task_initialization(void)
{
	sr_ecp_msg->message("ECP loaded");
}

void task::main_task_algorithm(void)
{
	int isTest = config.return_int_value("test_mode");

	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("lib::NEXT_STATE received");

		switch ( (ecp_mp::task::ECP_FESTIVAL_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
			case ecp_mp::task::ECP_GEN_FESTIVAL:
				if(isTest)
					sr_ecp_msg->message(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				else
				{
					fg->set_voice((generator::base::VOICE) mp_command.ecp_next_state.mp_2_ecp_next_state_variant);
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

}
} // namespace festival

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new festival::task::task(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

