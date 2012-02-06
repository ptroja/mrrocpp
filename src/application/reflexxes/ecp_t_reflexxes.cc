#include <memory>

#include "base/lib/sr/srlib.h"
#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "ecp_g_reflexxes.h"
#include "ecp_t_reflexxes.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

reflexxes::reflexxes(lib::configurator &_config) : common::task::task(_config)
{
	// the robot is choose dependendant on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6ot_m::robot (*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6p_m::robot (*this);
	} else {
		assert(0);
	}

	gen = std::unique_ptr<generator::reflexxes> (new generator::reflexxes(*this));

	sr_ecp_msg->message("ecp reflexxes loaded");
}

void reflexxes::main_task_algorithm(void)
{
	const double pose1[] = { 0, -0.068, -1.548, 0.012, 1.207, 4.173, -2.664 };
	const double pose2[] = { 0.10, 1.042, -1.748, -0.058, 0, 4.303, -2.754 };

	for(;;) {
		gen->set_goal_pose(pose1);
		gen->Move();

		gen->set_goal_pose(pose2);
		gen->Move();
	}
}

task_base* return_created_ecp_task (lib::configurator &_config)
{
	return new reflexxes(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
