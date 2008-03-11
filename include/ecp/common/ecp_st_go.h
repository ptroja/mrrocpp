// -------------------------------------------------------------------------
//                            ecp_st_go.h
// -------------------------------------------------------------------------

#if !defined(_ECP_SUB_TASK_GO_H)
#define _ECP_SUB_TASK_GO_H

#include "ecp/common/ecp_task.h"

void ecp_gripper_opening (ecp_task& ecp_object, double gripper_increment, int motion_time);

class ecp_sub_task_go : public ecp_sub_task
{

private:
    trajectory_description tdes;

    void init();

public:
    ecp_sub_task_go(ecp_task &_ecp_t);
    void configure(double gripper_increment, int motion_time);
    void execute();
};


#endif
