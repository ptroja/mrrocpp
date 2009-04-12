#if !defined(_ECP_T_RCSC_IRP6P_H)
#define _ECP_T_RCSC_IRP6P_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp/common/ecp_st_go.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {

class ecp_task_rcsc_irp6p: public common::task::ecp_task
{
protected:
    //generatory
	common::ecp_generator_t* gt;
	common::ecp_tff_nose_run_generator* nrg;
	common::ecp_tff_rubik_grab_generator* rgg;
	common::ecp_tff_gripper_approach_generator* gag;
	common::ecp_tff_rubik_face_rotate_generator* rfrg;
	common::ecp_teach_in_generator* tig;
	common::ecp_smooth_generator* sg;
	common::bias_edp_force_generator* befg;
	common::weight_meassure_generator* wmg;
    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

public:
    // KONSTRUKTORY
    ecp_task_rcsc_irp6p(configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void task_initialization(void);
    void main_task_algorithm(void);

};

} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
