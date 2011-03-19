//#include "generator/ecp/ecp_g_smooth.h"
//#include "generator/ecp/ecp_g_sleep.h"

//#include "subtask/ecp_st_bias_edp_force.h"
//#include "subtask/ecp_st_tff_nose_run.h"
//

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"

#include "base/ecp/ecp_task.h"
#include "ecp_t_gen_test.h"

#include "robot/irp6p_m/const_irp6p_m.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
gen_test::gen_test(lib::configurator &_config) :
	common::task::task(_config)
{
	// the robot is choose dependently on the section of configuration file sent as argv[4]
	if (config.section_name == lib::irp6ot_m::ECP_SECTION) {
		ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6ot_m::robot(*this);
	} else if (config.section_name == lib::irp6p_m::ECP_SECTION) {
		ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6p_m::robot(*this);
	} else if (config.section_name == lib::polycrank::ECP_SECTION) {
		ecp_m_robot = (boost::shared_ptr<robot_t>) new polycrank::robot(*this);
	} else if (config.section_name == lib::conveyor::ECP_SECTION) {
		ecp_m_robot = (boost::shared_ptr<robot_t>) new conveyor::robot(*this);
	} else {
		// TODO: throw
	}

	// utworzenie podzadan
	{
		sub_task::sub_task* ecpst;
		ecpst = new sub_task::sub_task_const_vel_gen_test(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_CONST_VEL_GEN_TEST] = ecpst;

		ecpst = new sub_task::sub_task_smooth_gen_test(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_GEN_TEST] = ecpst;
	}

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
