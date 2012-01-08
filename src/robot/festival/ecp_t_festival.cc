#include <cstdio>
#include <cstring>
#include <unistd.h>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
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
	int isTest = config.value <int>("test_mode");

	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("NEXT_STATE received");

		if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_FESTIVAL) {

			if (isTest)
				sr_ecp_msg->message(reinterpret_cast <const char*>(mp_command.ecp_next_state.sg_buf.data));
			else {
				fg.set_voice((generator::generator::VOICE) mp_command.ecp_next_state.variant);
				fg.set_phrase(mp_command.ecp_next_state.sg_buf.get <std::string>());
				fg.Move();
			}

		}

		termination_notice();
	}
}

}
} // namespace festival

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new festival::task::task(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

