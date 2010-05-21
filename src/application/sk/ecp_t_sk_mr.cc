#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp_t_sk_mr_test.h"

#include "ecp/sk_mr/ecp_r_sk_mr.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp/common/generator/ecp_g_sleep.h"
#include "ecp_g_sk_mr_test.h"
#include "ecp_t_sk_mr_test.h"

namespace mrrocpp {
namespace ecp {
namespace sk_mr {
namespace task {

// KONSTRUKTORY
sk_mr_test::sk_mr_test(lib::configurator &_config) :
	task(_config) {
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = new robot(*this);

	gt = new common::generator::transparent(*this);
	g_sleep = new common::generator::sleep(*this);
	g_sk_mr = new common::generator::sk_mr(*this);

	sr_ecp_msg->message("ECP BIRD HAND TEST loaded");
}

void sk_mr_test::main_task_algorithm(void) {
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");
		//printf("postument: %d\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
		flushall();

		switch ((ecp_mp::task::SK_MR_TEST_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
		case ecp_mp::task::ECP_GEN_TRANSPARENT:
			gt->throw_kinematics_exceptions
					= (bool) mp_command.ecp_next_state.mp_2_ecp_next_state_variant;
			gt->Move();
			break;
		case ecp_mp::task::ECP_GEN_SLEEP:
			g_sleep->init_time(
					mp_command.ecp_next_state.mp_2_ecp_next_state_variant);
			g_sleep->Move();
			break;
		case ecp_mp::task::ECP_GEN_SK_MR: {
			sr_ecp_msg->message("ECP_GEN_SK_MR");

			g_sk_mr->Move();
			break;
		}
		default:
			break;
		} // end switch

		ecp_termination_notice();
	} //end for
}

}
} // namespace sk_mr

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new sk_mr::task::sk_mr_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
