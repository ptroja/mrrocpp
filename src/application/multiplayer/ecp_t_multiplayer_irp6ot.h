#if !defined(_ECP_T_MULTIPLAYER_IRP6OT_H)
#define _ECP_T_MULTIPLAYER_IRP6OT_H

#include "base/ecp/ecp_task.h"
#include "subtask/ecp_st_go.h"
#include "base/ecp/ecp_g_transparent.h"
#include "generator/ecp/force/ecp_g_weight_measure.h"
#include "generator/ecp/force/ecp_g_tff_rubik_grab.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

class multiplayer : public common::task::task
{
private:
	//generatory
	common::generator::transparent* gt;
	//common::generator::smooth* sg;

	common::generator::weight_measure* wmg;

	//podzadania
	common::sub_task::gripper_opening* go_st;

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
