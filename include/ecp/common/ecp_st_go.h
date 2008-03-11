// -------------------------------------------------------------------------
//                            ecp_st_go.h
// -------------------------------------------------------------------------

#if !defined(_ECP_SUB_TASK_GO_H)
#define _ECP_SUB_TASK_GO_H

#include "ecp/common/ecp_task.h"

class ecp_sub_task_gripper_opening : public ecp_sub_task
{

private:
    trajectory_description tdes;

    void init();

public:
    ecp_sub_task_gripper_opening(ecp_task &_ecp_t);
    void configure(double gripper_increment, int motion_time);
    void execute();
};


#endif
