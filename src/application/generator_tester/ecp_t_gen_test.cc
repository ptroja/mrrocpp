//#include "generator/ecp/ecp_g_smooth.h"
//#include "generator/ecp/ecp_g_sleep.h"

//#include "subtask/ecp_st_bias_edp_force.h"
//#include "subtask/ecp_st_tff_nose_run.h"
//

#include "base/ecp/ecp_task.h"
#include "ecp_t_gen_test.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
gen_test::gen_test(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependently on the section of configuration file sent as argv[4]
	if (config.section_name == lib::irp6ot_m::ECP_SECTION) {
		ecp_m_robot = new irp6ot_m::robot(*this);
	} else if (config.section_name == lib::irp6p_m::ECP_SECTION) {
		ecp_m_robot = new irp6p_m::robot(*this);
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

task* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::gen_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
