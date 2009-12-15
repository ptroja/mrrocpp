#if !defined(_ECP_T_RCSC_IRP6OT_H)
#define _ECP_T_RCSC_IRP6OT_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/generator/ecp_g_t.h"
#include "ecp/common/task/ecp_st_go.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
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
	common::ecp_teach_in_generator* tig;
	common::generator::smooth* sg;
	common::generator::smooth2* sg2;
	common::generator::bias_edp_force* befg;
	common::generator::weight_meassure* wmg;
    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

public:
    rcsc(lib::configurator &_config);
    ~rcsc();

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
