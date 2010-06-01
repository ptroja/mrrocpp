#if !defined(_ECP_T_RCSC_IRP6P_H)
#define _ECP_T_RCSC_IRP6P_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_transparent.h"
#include "ecp/common/task/ecp_st_go.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace task {

class rcsc: public common::task::task
{
protected:
    //generatory
	common::generator::transparent* gt;
	common::generator::tff_nose_run* nrg;
	common::generator::tff_rubik_grab* rgg;
	common::generator::tff_gripper_approach* gag;
	common::generator::tff_rubik_face_rotate* rfrg;
	common::generator::teach_in* tig;
	common::generator::smooth* sg;
	common::generator::bias_edp_force* befg;
	common::generator::weight_meassure* wmg;
    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

public:
    // KONSTRUKTORY
    rcsc(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
