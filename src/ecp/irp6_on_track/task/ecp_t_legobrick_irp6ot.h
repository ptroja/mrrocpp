#if !defined(_ECP_T_LEGOBRICK_IRP6OT_H)
#define _ECP_T_LEGOBRICK_IRP6OT_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_smooth.h"
//#include "ecp/common/generator/ecp_g_t.h"
//#include "ecp/common/task/ecp_st_go.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class lego_brick: public common::task::task
{
protected:
    //generatory
    /*ecp_generator_t* gt;
    ecp_tff_nose_run_generator* nrg;
    ecp_tff_rubik_grab_generator* rgg;
    ecp_tff_gripper_approach_generator* gag;
    ecp_tff_rubik_face_rotate_generator* rfrg;
    ecp_g_teach_in* tig;*/
	common::generator::smooth* sg;
    //bias_edp_force_generator* befg;
    //weight_meassure_generator* wmg;
    //podzadania
    //ecp_sub_task_gripper_opening* go_st;
public:
    lego_brick(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
