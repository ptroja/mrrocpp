#if !defined(_ECP_T_RCSC_IRP6OT_H)
#define _ECP_T_RCSC_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp/common/ecp_t_rcsc.h"

class ecp_task_rcsc_irp6ot: public ecp_task
{
protected:
    ecp_generator_t* gt;
    ecp_tff_nose_run_generator* nrg;
    ecp_tff_rubik_grab_generator* rgg;
    ecp_tff_gripper_approach_generator* gag;
    ecp_tff_rubik_face_rotate_generator* rfrg;
    ecp_teach_in_generator* tig;
    ecp_smooth_generator* sg;
    bias_edp_force_generator* befg;
    weight_meassure_generator* wmg;

public:
    ecp_task_rcsc_irp6ot(configurator &_config);
    ~ecp_task_rcsc_irp6ot();

    // methods for ECP template to redefine in concrete classes
    void task_initialization(void);
    void main_task_algorithm(void);
};

#endif
