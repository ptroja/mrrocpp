// -------------------------------------------------------------------------
//                            ecp_st_go.h
// -------------------------------------------------------------------------

#if !defined(_ECP_SUB_TASK_GO_H)
#define _ECP_SUB_TASK_GO_H

#include "base/ecp/ecp_sub_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace sub_task {

class gripper_opening : public sub_task
{

private:
	lib::trajectory_description tdes;

	void init();

public:
	gripper_opening(task::task &_ecp_t);
	void configure(double gripper_increment, int motion_time);
	void execute();
	void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
