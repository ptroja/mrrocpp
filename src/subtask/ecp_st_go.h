// -------------------------------------------------------------------------
//                            ecp_st_go.h
// -------------------------------------------------------------------------

#if !defined(_ECP_SUB_TASK_GO_H)
#define _ECP_SUB_TASK_GO_H

#include "base/ecp/ecp_task.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class sub_task_gripper_opening : public sub_task
{

private:
    lib::trajectory_description tdes;

    void init();

public:
    sub_task_gripper_opening(task &_ecp_t);
    void configure(double gripper_increment, int motion_time);
    void execute();
    void conditional_execution();
};

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp


#endif
