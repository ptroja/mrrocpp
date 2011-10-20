#if !defined(_ECP_T_BLOCK_MOVE_H)
#define _ECP_T_BLOCK_MOVE_H

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

//#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "generator/ecp/force/ecp_g_tff_gripper_approach.h"

#include "base/ecp/ecp_task.h"
#include "subtask/ecp_mp_st_smooth_file_from_mp.h"
#include "subtask/ecp_st_go.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class block_move : public common::task::task
{
protected:
	common::generator::tff_gripper_approach* gtga;
	//common::sub_task::gripper_opening* stgo;

public:

	block_move(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);

};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
