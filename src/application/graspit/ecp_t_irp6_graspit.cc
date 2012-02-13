#include <cstdio>
#include <string>
#include <unistd.h>
#include <iostream>

#include "base/lib/configurator.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "generator/ecp/constant_velocity/ecp_g_constant_velocity.h"
#include "ecp_t_irp6_graspit.h"
#include "ecp_mp_t_graspit.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
irp6_grasp::irp6_grasp(lib::configurator &_config) :
	common::task::task(_config)
{
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6p_m::robot(*this);
	}

	cvgenjoint = new generator::constant_velocity(*this, lib::ECP_JOINT, 6);
	cvgenjoint->set_debug(true);

	sr_ecp_msg->message("ecp IRP6 loaded");
}

void irp6_grasp::main_task_algorithm(void)
{

	sr_ecp_msg->message("ecp IRP6 ready");

	struct _irp6
	{
		double joint[6];
	} mp_ecp_irp6_command;
	std::vector <double> coordinates1(7);

	std::stringstream ss(std::stringstream::in | std::stringstream::out);

	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");
		flushall();

		if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_IRP6) {

			sr_ecp_msg->message("ECP_GEN_IRP6");

			memcpy(&mp_ecp_irp6_command, mp_command.ecp_next_state.sg_buf.data, sizeof(mp_ecp_irp6_command));
			//ignore first DOF of IRp6_on_track, not used in GraspIt
			coordinates1[0] = 0.0;
			for (int i = 0; i < 6; ++i)
				coordinates1[i+1] = mp_ecp_irp6_command.joint[i];

			cvgenjoint->reset();
			cvgenjoint->set_absolute();
			cvgenjoint->load_absolute_joint_trajectory_pose(coordinates1);
			if (cvgenjoint->calculate_interpolate())
				cvgenjoint->Move();
		}

		termination_notice();
	}

	termination_notice();
}

} // namespace task
} // namespace common

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::irp6_grasp(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp

