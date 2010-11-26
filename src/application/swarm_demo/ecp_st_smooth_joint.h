#if !defined(_ECP_SUB_TASK_SMOOTH_JOINT_H)
#define _ECP_SUB_TASK_SMOOTH_JOINT_H

#include "base/ecp/ecp_sub_task.h"
#include "application/swarm_demo/ecp_mp_st_smooth_joint.h"

namespace mrrocpp {
namespace ecp {
namespace common {

namespace generator {
class newsmooth;
}

namespace sub_task {

class sub_task_smooth_joint : public sub_task
{

private:
	generator::newsmooth * sgenjoint;
	std::string path;

public:
	sub_task_smooth_joint(task::task & _ecp_t);
	~sub_task_smooth_joint();

	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
