#include "base/lib/sr/srlib.h"

#include "robot/shead/ecp_r_shead1.h"
#include "robot/shead/ecp_r_shead2.h"

#include "ecp_t_shead.h"
#include "ecp_g_shead.h"
#include "ecp_mp_g_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace task {

swarmitfix::swarmitfix(lib::configurator &_config) :
		common::task::_task <ecp::shead::robot>(_config)
{
	// Robot is choosen dependending on the section of configuration file sent as argv[4].
	if (config.robot_name == lib::shead1::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new shead1::robot(*this);
	} else if (config.robot_name == lib::shead2::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new shead2::robot(*this);
	} else {
		// TODO: throw
	}

	// Create the generators

	g_joint_epos_command = new shead::generator::joint_epos_command(*this);

	sr_ecp_msg->message("ecp shead transparent loaded");
}

void swarmitfix::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::shead::generator::ECP_JOINT_EPOS_COMMAND) {

		g_joint_epos_command->Move();

	}

}

}
} // namespace shead

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new shead::task::swarmitfix(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
