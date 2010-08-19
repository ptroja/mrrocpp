#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "robot/festival/ecp_g_festival.h"
#include "robot/festival/ecp_mp_t_festival.h"
#include "robot/festival/ecp_t_festival.h"

namespace mrrocpp {
namespace ecp {
namespace festival {
namespace task {

// KONSTRUKTORY
task::task(lib::configurator &_config) :
	common::task::task(_config), fg(*this)
{
}

void task::main_task_algorithm(void)
{
	int isTest = config.value <int> ("test_mode");

	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("NEXT_STATE received");

		if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_FESTIVAL) {

			if (isTest)
				sr_ecp_msg->message(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			else {
				fg.set_voice((generator::generator::VOICE) mp_command.ecp_next_state.mp_2_ecp_next_state_variant);
				fg.set_phrase(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
				fg.Move();
			}

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

