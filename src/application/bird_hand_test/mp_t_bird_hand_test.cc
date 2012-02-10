// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"

#include "ecp_mp_t_bird_hand_test.h"
#include "mp_t_bird_hand_test.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/bird_hand/dp_bird_hand.h"
#include "robot/bird_hand/const_bird_hand.h"
#include "generator/ecp/transparent/ecp_mp_g_transparent.h"
#include "ecp_mp_g_bird_hand_test.h"

#include "robot/bird_hand/mp_r_bird_hand.h"

#include <iostream>
#include <string>
#include <sstream>

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new bird_hand_test(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void bird_hand_test::create_robots()
{
	ACTIVATE_MP_ROBOT(bird_hand);

}

bird_hand_test::bird_hand_test(lib::configurator &_config) :
	task(_config)
{
}

void bird_hand_test::main_task_algorithm(void)
{

	sr_ecp_msg->message("New bird_hand_test series");

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, (int) 0, "", lib::bird_hand::ROBOT_NAME);

	send_end_motion_to_ecps(lib::bird_hand::ROBOT_NAME);

	sr_ecp_msg->message("4");

	set_next_ecp_state(ecp_mp::bird_hand::generator::ECP_GEN_BIRD_HAND_TEST, (int) 5, "", lib::bird_hand::ROBOT_NAME);
	sr_ecp_msg->message("5");
	wait_for_task_termination(false, lib::bird_hand::ROBOT_NAME);

	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
