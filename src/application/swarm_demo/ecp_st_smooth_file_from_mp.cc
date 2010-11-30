#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "application/swarm_demo/ecp_st_smooth_file_from_mp.h"
#include "generator/ecp/ecp_g_newsmooth.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

sub_task_smooth_file_from_mp::sub_task_smooth_file_from_mp(task::task & _ecp_t, lib::ECP_POSE_SPECIFICATION pose_spec, int coordinates_nr) :
	sub_task(_ecp_t)
{

	if (_ecp_t.ecp_m_robot->robot_name == lib::irp6p_m::ROBOT_NAME) {
		sgen = new generator::newsmooth(ecp_t, lib::ECP_JOINT, 6);
		sgen->set_absolute();
		sgen->set_debug(true);

	} else if (_ecp_t.ecp_m_robot->robot_name == lib::irp6ot_m::ROBOT_NAME) {
		sgen = new generator::newsmooth(ecp_t, lib::ECP_JOINT, 7);
		sgen->set_debug(true);
	}

}

void sub_task_smooth_file_from_mp::conditional_execution()
{

	path = std::string("");
	sgen->reset();
	path += ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string;
	sgen->load_trajectory_from_file(path.c_str());

	if (sgen->calculate_interpolate() && sgen->detect_jerks(1) == 0) {
		sgen->Move();
	}
}

sub_task_smooth_file_from_mp::~sub_task_smooth_file_from_mp()
{
	delete sgen;
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
