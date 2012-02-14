#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "base/ecp/ecp_task.h"
#include "ecp_t_gen_test.h"

#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

gen_test::gen_test(lib::configurator &_config) :
		common::task::task(_config)
{
	// the robot is choose dependently on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else if (config.robot_name == lib::conveyor::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new conveyor::robot(*this);
	} else {
		// TODO: throw
	}



	// TEMPORARY REMOVAL
        //register_generator(new common::generator::spline_gen_test(*this));
        register_generator(new common::generator::smooth_gen_test(*this));
        //register_generator(new common::generator::const_vel_gen_test(*this));
	sr_ecp_msg->message("ecp GEN_TEST loaded");
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::gen_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
