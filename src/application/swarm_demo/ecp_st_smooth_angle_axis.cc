#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "application/swarm_demo/ecp_st_smooth_angle_axis.h"
#include "generator/ecp/ecp_g_newsmooth.h"

#include "robot/irp6ot_m/const_irp6ot_m.h"
#include "robot/irp6p_m/const_irp6p_m.h"

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

sub_task_smooth_angle_axis::sub_task_smooth_angle_axis(task::task & _ecp_t) :
	sub_task(_ecp_t)
{

	sgenangle = new generator::newsmooth(ecp_t, lib::ECP_XYZ_ANGLE_AXIS, 6);
	sgenangle->set_absolute();
	sgenangle->set_debug(true);

}

void sub_task_smooth_angle_axis::conditional_execution()
{

	path = std::string("");
	sgenangle->reset();
	path += ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string;
	sgenangle->load_trajectory_from_file(path.c_str());


	if (sgenangle->calculate_interpolate() && sgenangle->detect_jerks(1) == 0) {
		sgenangle->Move();
	}
}

sub_task_smooth_angle_axis::~sub_task_smooth_angle_axis()
{
	delete sgenangle;
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
