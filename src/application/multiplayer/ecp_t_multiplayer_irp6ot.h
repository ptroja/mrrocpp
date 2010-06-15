#if !defined(_ECP_T_MULTIPLAYER_IRP6OT_H)
#define _ECP_T_MULTIPLAYER_IRP6OT_H

#include "ecp/common/task/ecp_task.h"
#include "ecp/common/task/ecp_st_go.h"
#include "ecp/common/generator/ecp_g_transparent.h"

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

	common::generator::weight_meassure* wmg;

    //podzadania
	common::task::ecp_sub_task_gripper_opening* go_st;

   	common::generator::tff_rubik_grab *rgg;


public:
    multiplayer(lib::configurator &_config);

    // methods for ECP template to redefine in concrete classes
    void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6ot
} // namespace ecp
} // namespace mrrocpp

#endif
