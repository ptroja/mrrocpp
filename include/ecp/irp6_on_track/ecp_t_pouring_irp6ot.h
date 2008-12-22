#if !defined(_ECP_T_POURING_IRP6OT_H)
#define _ECP_T_POURING_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_generator_t.h"

class ecp_task_pouring_irp6ot: public ecp_task
{
protected:
    ecp_smooth_generator* sg;
    ecp_tool_change_generator* tcg;
    //podzadania
    ecp_sub_task_gripper_opening* go_st;

public:
    // KONSTRUKTORY
    ecp_task_pouring_irp6ot(configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void task_initialization(void);
    void main_task_algorithm(void);
    void grip(double gripper_increment, int motion_time);

};

#endif
