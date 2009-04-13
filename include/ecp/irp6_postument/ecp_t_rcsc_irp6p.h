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
	common::generator::ecp_generator_t* gt;
	common::generator::ecp_tff_nose_run_generator* nrg;
	common::generator::ecp_tff_rubik_grab_generator* rgg;
	common::generator::ecp_tff_gripper_approach_generator* gag;
	common::generator::ecp_tff_rubik_face_rotate_generator* rfrg;
	common::ecp_teach_in_generator* tig;
	common::generator::ecp_smooth_generator* sg;
	common::generator::bias_edp_force_generator* befg;
	common::generator::weight_meassure_generator* wmg;
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
