#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/ecp_r_irp6p_tfg.h"

#include "robot/irp6_tfg/ecp_t_tfg.h"
#include "generator/ecp/ecp_mp_g_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace irp6_tfg {
namespace task {

// KONSTRUKTORY
tfg::tfg(lib::configurator &_config) :
	common::task::task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_tfg::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_tfg::robot(*this);
	} else if (config.robot_name == lib::irp6p_tfg::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <ecp::common::robot::ecp_robot>) new irp6p_tfg::robot(*this);
	} else {
		// TODO: throw
	}

	tfgg = new generator::tfg(*this, 10);

	sr_ecp_msg->message("ecp TFG loaded");
}

void tfg::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFG) {

		tfgg->Move();
	}

}

}
} // namespace common

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6_tfg::task::tfg(_config);
}

}
} // namespace irp6_tfg
} // namespace ecp
} // namespace mrrocpp
