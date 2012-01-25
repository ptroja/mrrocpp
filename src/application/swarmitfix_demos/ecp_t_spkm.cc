#include "base/lib/sr/srlib.h"

#include "robot/spkm/ecp_r_spkm1.h"
#include "robot/spkm/ecp_r_spkm2.h"

#include "ecp_t_spkm.h"
#include "ecp_g_spkm.h"
#include "ecp_mp_g_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace task {

swarmitfix::swarmitfix(lib::configurator &_config) :
		common::task::_task <ecp::spkm::robot>(_config)
{
	// Robot is choosen dependending on the section of configuration file sent as argv[4].
	if (config.robot_name == lib::spkm1::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new spkm1::robot(*this);
	} else if (config.robot_name == lib::spkm2::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new spkm2::robot(*this);
	} else {
		// TODO: throw
	}

	// Create the generators

	g_joint_epos_command = new spkm::generator::joint_epos_command(*this);
	g_external_epos_command = new spkm::generator::external_epos_command(*this);

	sr_ecp_msg->message("ecp spkm transparent loaded");
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
