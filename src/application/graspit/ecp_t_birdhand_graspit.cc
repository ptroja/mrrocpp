#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "ecp_mp_t_graspit.h"
#include "ecp_mp_g_birdhand.h"

#include "robot/bird_hand/ecp_r_bird_hand.h"
#include "ecp_g_birdhand_graspit.h"
#include "ecp_t_birdhand_graspit.h"

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

	g_bird_hand = new common::generator::bird_hand(*this);

	sr_ecp_msg->message("ecp BIRDHAND loaded");
}

void bird_hand_test::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_BIRD_HAND) {

		sr_ecp_msg->message("ECP_GEN_BIRD_HAND");

		g_bird_hand->Move();

	} // end switch


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
