// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "mp/mp.h"
#include "ecp_mp_t_sk_mr_test.h"
#include "mp_t_sk_mr_test.h"
#include "lib/single_thread_port.h"
#include "lib/mrmath/mrmath.h"
#include "lib/data_port_headers/sk_mr.h"

#include <iostream>
#include <string>
#include <sstream>

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config) {
	return new sk_mr_test(_config);
}

sk_mr_test::sk_mr_test(lib::configurator &_config) :
	task(_config) {
}

void sk_mr_test::main_task_algorithm(void) {

	sr_ecp_msg->message("New sk_mr_test series");

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_TRANSPARENT, (int) 0, "",
			0, 1, lib::ROBOT_SK_MR);

	send_end_motion_to_ecps(1, lib::ROBOT_SK_MR);

	sr_ecp_msg->message("4");

	set_next_ecps_state((int) ecp_mp::task::ECP_GEN_SK_MR, (int) 5, "", 0, 1,
			lib::ROBOT_SK_MR);
	sr_ecp_msg->message("5");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
			1, 1, lib::ROBOT_SK_MR, lib::ROBOT_SK_MR);

	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
