#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "ecp_mp_t_bird_hand_test.h"

#include "robot/bird_hand/ecp_r_bird_hand.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_sleep.h"
#include "ecp_g_bird_hand_test.h"
#include "ecp_t_bird_hand_test.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "generator/ecp/ecp_mp_g_sleep.h"
#include "ecp_mp_g_bird_hand_test.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {
namespace task {

// KONSTRUKTORY
bird_hand_test::bird_hand_test(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = new robot(*this);

	gt = new common::generator::transparent(*this);
	g_sleep = new common::generator::sleep(*this);
	g_bird_hand = new generator::bird_hand(*this);

	sr_ecp_msg->message("ecp BIRD HAND TEST loaded");
}

void bird_hand_test::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TRANSPARENT) {

		gt->throw_kinematics_exceptions = (bool) mp_command.ecp_next_state.mp_2_ecp_next_state_variant;
		gt->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_SLEEP) {

		g_sleep->init_time(mp_command.ecp_next_state.mp_2_ecp_next_state_variant);
		g_sleep->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::bird_hand::generator::ECP_GEN_BIRD_HAND_TEST) {

		sr_ecp_msg->message("ECP_GEN_BIRD_HAND");

		g_bird_hand->Move();
	}

}

}
} // namespace bird_hand

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new bird_hand::task::bird_hand_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
