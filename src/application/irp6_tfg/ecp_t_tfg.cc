#include "ecp_t_tfg.h"

// generators to be register headers
#include "ecp_g_tfg.h"
#include "ecp_g_constant_velocity_tfg.h"

// ecp_robots headers
#include "robot/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/ecp_r_irp6p_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace irp6_tfg {
namespace task {

tfg::tfg(lib::configurator &_config) :
		common::task::task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_tfg::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_tfg::robot(*this);
	} else if (config.robot_name == lib::irp6p_tfg::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <ecp::common::robot::ecp_robot>) new irp6p_tfg::robot(*this);
	} else {
		throw std::runtime_error("Robot not supported");
	}

	// generator registration
	register_generator(new generator::tfg(*this, 10));
	register_generator(new generator::constant_velocity(*this, lib::ECP_JOINT, 1));

	sr_ecp_msg->message("ecp loaded");
}

}
} // namespace irp6_tfg

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
