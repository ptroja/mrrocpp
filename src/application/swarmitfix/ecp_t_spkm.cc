#include <cstdio>

#include "base/lib/sr/srlib.h"

#include "robot/spkm/ecp_r_spkm.h"

#include "ecp_t_spkm.h"
#include "ecp_g_spkm.h"
#include "ecp_mp_g_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace task {

// KONSTRUKTORY
swarmitfix::swarmitfix(lib::configurator &_config) :
	task_t(_config)
{
	// Create the robot object
	ecp_m_robot = (boost::shared_ptr<robot_t>) new robot(*this);

	// Create the generators
	g_pose = new generator::spkm_pose(*this);
	g_quickstop = new generator::spkm_quickstop(*this);

	sr_ecp_msg->message("ecp spkm loaded");
}

void swarmitfix::mp_2_ecp_next_state_string_handler(void)
{
	if (mp_2_ecp_next_state_string == ecp_mp::spkm::generator::ECP_GEN_QUICKSTOP) {

		g_quickstop->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::spkm::generator::ECP_GEN_POSE) {

		// g_pose->init_time(mp_command.ecp_next_state.mp_2_ecp_next_state_variant);

		g_pose->Move();

	}
}

}
} // namespace spkm

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new spkm::task::swarmitfix(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
