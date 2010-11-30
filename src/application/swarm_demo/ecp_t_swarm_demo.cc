#include "base/ecp/ecp_task.h"
#include "ecp_t_swarm_demo.h"
#include "subtask/ecp_st_smooth_file_from_mp.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "subtask/ecp_st_bias_edp_force.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
swarm_demo::swarm_demo(lib::configurator &_config) :
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
		ecpst = new sub_task::bias_edp_force(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = ecpst;
	}

	{
		sub_task::sub_task_smooth_file_from_mp* ecpst;

		ecpst = new sub_task::sub_task_smooth_file_from_mp(*this, lib::ECP_JOINT);
		subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP] = ecpst;

		ecpst = new sub_task::sub_task_smooth_file_from_mp(*this, lib::ECP_XYZ_ANGLE_AXIS);
		subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP] = ecpst;
	}

	sr_ecp_msg->message("ecp SWARM DEMO loaded");
}

task* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::swarm_demo(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
