#if !defined(_ECP_T_RCSC_IRP6P_H)
#define _ECP_T_RCSC_IRP6P_H

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_g_transparent.h"
#include "generator/ecp/force/ecp_g_weight_measure.h"
#include "generator/ecp/force/ecp_g_tff_rubik_grab.h"
#include "generator/ecp/force/ecp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"
#include "subtask/ecp_st_go.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace task {

class rcsc : public common::task::task
{
protected:
	//generatory
	common::generator::transparent* gt;

	common::generator::tff_rubik_grab* rgg;
	common::generator::tff_gripper_approach* gag;
	common::generator::tff_rubik_face_rotate* rfrg;
	common::generator::teach_in* tig;
	//common::generator::smooth* sg;

	common::generator::weight_measure* wmg;
	//podzadania
	common::sub_task::gripper_opening* go_st;

public:
	// KONSTRUKTORY
	rcsc(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
