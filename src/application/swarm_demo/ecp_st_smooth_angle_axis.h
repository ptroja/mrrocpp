#if !defined(_ECP_SUB_TASK_SMOOTH_ANGLE_AXIS_H)
#define _ECP_SUB_TASK_SMOOTH_ANGLE_AXIS_H

#include "base/ecp/ecp_sub_task.h"
#include "application/swarm_demo/ecp_mp_st_smooth_angle_axis.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class newsmooth;
}

namespace sub_task {

class sub_task_smooth_angle_axis : public sub_task
{

private:
	generator::newsmooth * sgenangle;
	std::string path;

public:
	sub_task_smooth_angle_axis(task::task & _ecp_t);
	~sub_task_smooth_angle_axis();

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
