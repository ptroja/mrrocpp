#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "robot/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/ecp_r_irp6p_tfg.h"

#include "ecp_t_tfg.h"
#include "ecp_mp_g_tfg.h"
#include "generator/ecp/ecp_mp_g_constant_velocity.h"
#include "vector"

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

	register_sg(new generator::tfg(*this, 10));

	cvg = new common::generator::constant_velocity(*this, lib::ECP_JOINT, 1);
	cvg->set_debug(true);

	sr_ecp_msg->message("ecp TFG loaded");
}

void tfg::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_CONSTANT_VELOCITY) {

		cvg->reset();
		std::vector <double> pos(1, mp_command.ecp_next_state.sg_buf.get <double>());
		std::vector <double> joint_velocity(1, 0.003);

		cvg->set_joint_velocity_vector(joint_velocity);

		switch ((lib::MOTION_TYPE) mp_command.ecp_next_state.variant)
		{
			case lib::RELATIVE:
				cvg->set_relative();
				cvg->load_relative_joint_trajectory_pose(pos);
				break;
			case lib::ABSOLUTE:
				cvg->set_absolute();
				cvg->load_absolute_joint_trajectory_pose(pos);
				break;
			default:
				break;
		}

		cvg->calculate_interpolate();
		cvg->Move();
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
