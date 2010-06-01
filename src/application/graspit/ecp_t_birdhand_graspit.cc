#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp_t_graspit.h"

#include "ecp/bird_hand/ecp_r_bird_hand.h"
#include "ecp_g_birdhand_graspit.h"
#include "ecp_t_birdhand_graspit.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {
namespace task {

// KONSTRUKTORY
bird_hand_test::bird_hand_test(lib::configurator &_config) :
	task(_config) {
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = new robot(*this);

	g_bird_hand = new common::generator::bird_hand(*this);

	sr_ecp_msg->message("ECP BIRDHAND loaded");
}

void bird_hand_test::main_task_algorithm(void) {

	sr_ecp_msg->message("ECP BIRDHAND ready");

	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");
		//printf("postument: %d\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
		flushall();

		switch ((ecp_mp::task::GRASPIT_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
		case ecp_mp::task::ECP_GEN_BIRD_HAND: {
			sr_ecp_msg->message("ECP_GEN_BIRD_HAND");

			g_bird_hand->Move();
			break;
		}
		default:
			break;
		} // end switch

		ecp_termination_notice();
	} //end for
}

}
} // namespace bird_hand

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new bird_hand::task::bird_hand_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
