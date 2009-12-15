#if !defined(_ECP_T_MULTIPLAYER_IRP6OT_H)
#define _ECP_T_MULTIPLAYER_IRP6OT_H

#include "ecp/common/ecp_task.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/common/ecp_g_t.h"
#include "ecp/irp6_on_track/ecp_g_vis_sac_lx.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class multiplayer : public common::task::task
{
private:
	//generatory
	common::generator::transparent* gt;
	common::generator::smooth* sg;
	common::generator::bias_edp_force* befg;
	common::generator::weight_meassure* wmg;

    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

	generator::vis_sac_lx *takeg;
   	common::generator::tff_rubik_grab *rgg;


public:
    multiplayer(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void main_task_algorithm(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
