#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "ecp_mp_t_bird_hand_test.h"

#include "robot/bird_hand/ecp_r_bird_hand.h"
#include "generator/ecp/newsmooth/ecp_g_newsmooth.h"
#include "generator/ecp/sleep/ecp_g_sleep.h"
#include "ecp_g_bird_hand_test.h"
#include "ecp_t_bird_hand_test.h"
#include "generator/ecp/transparent/ecp_mp_g_transparent.h"
#include "generator/ecp/sleep/ecp_mp_g_sleep.h"
#include "ecp_mp_g_bird_hand_test.h"

namespace mrrocpp {
namespace ecp {
namespace bird_hand {
namespace task {

// KONSTRUKTORY
bird_hand_test::bird_hand_test(lib::configurator &_config) :
		common::task::_task <ecp::bird_hand::robot>(_config)
//		common::task::task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = (boost::shared_ptr <robot_t>) new robot(*this);

	register_generator(new common::generator::sleep(*this));
	g_bird_hand = new generator::bird_hand(*this);

	sr_ecp_msg->message("ecp BIRD HAND TEST loaded");
}

void bird_hand_test::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::bird_hand::generator::ECP_GEN_BIRD_HAND_TEST) {

		sr_ecp_msg->message("ECP_GEN_BIRD_HAND");

		g_bird_hand->Move();
	}

}

}
} // namespace bird_hand

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new bird_hand::task::bird_hand_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
