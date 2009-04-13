#if !defined(_ECP_T_RCSC_IRP6P_H)
#define _ECP_T_RCSC_IRP6P_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_generator_t.h"
#include "ecp/common/ecp_st_go.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p {
namespace task {

class ecp_task_rcsc_irp6p: public common::task::base
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
	common::generator::bias_edp_force* befg;
	common::generator::weight_meassure* wmg;
    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

public:
    // KONSTRUKTORY
    ecp_task_rcsc_irp6p(configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void task_initialization(void);
    void main_task_algorithm(void);

};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
