#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "application/swarm_demo/ecp_st_smooth_joint.h"
#include "generator/ecp/ecp_g_newsmooth.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

sub_task_smooth_joint::sub_task_smooth_joint(task::task & _ecp_t) :
	sub_task(_ecp_t)
{

	if (_ecp_t.ecp_m_robot->robot_name == lib::irp6p_m::ROBOT_NAME) {
		sgenjoint = new generator::newsmooth(ecp_t, lib::ECP_JOINT, 6);
		std::vector <double> coordinates1(6);
		sgenjoint->set_absolute();
		sgenjoint->set_debug(true);

	} else if (_ecp_t.ecp_m_robot->robot_name == lib::irp6ot_m::ROBOT_NAME) {
		sgenjoint = new generator::newsmooth(ecp_t, lib::ECP_JOINT, 7);
		std::vector <double> coordinates2(7);
		sgenjoint->set_debug(true);
	}

}

void sub_task_smooth_joint::conditional_execution()
{

	path = std::string("");
	sgenjoint->reset();
	path += ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string;
	sgenjoint->load_trajectory_from_file(path.c_str());


	if (sgenjoint->calculate_interpolate() && sgenjoint->detect_jerks(1) == 0) {
		sgenjoint->Move();
	}
	// JOINT ABSOLUTE END
}

sub_task_smooth_joint::~sub_task_smooth_joint()
{
	delete sgenjoint;
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
