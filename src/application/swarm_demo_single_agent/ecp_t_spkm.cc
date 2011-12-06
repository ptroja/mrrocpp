#include "base/lib/sr/srlib.h"

#include "robot/spkm/ecp_r_spkm2.h"

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
	ecp_m_robot = (boost::shared_ptr <robot_t>) new spkm2::robot(*this);

	// Create the generators

	g_joint_epos_command = new spkm::generator::joint_epos_command(*this);
	g_external_epos_command = new spkm::generator::external_epos_command(*this);

	sr_ecp_msg->message("ecp spkm swarm demo single agent loaded");
}

void swarmitfix::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::spkm::generator::ECP_JOINT_EPOS_COMMAND) {

		g_joint_epos_command->Move();

	}

	if (mp_2_ecp_next_state_string == ecp_mp::spkm::generator::ECP_EXTERNAL_EPOS_COMMAND) {

		g_external_epos_command->Move();

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
