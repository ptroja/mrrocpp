#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_tfg.h"

#include "ecp/irp6ot_tfg/ecp_r_irp6ot_tfg.h"
#include "ecp/irp6p_tfg/ecp_r_irp6p_tfg.h"

#include "ecp/irp6_tfg/ecp_t_tfg.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
tfg::tfg(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	if (config.section_name == ECP_IRP6OT_TFG_SECTION) {
		ecp_m_robot = new irp6ot_tfg::robot(*this);
	} else if (config.section_name == ECP_IRP6P_TFG_SECTION) {
		ecp_m_robot = new irp6p_tfg::robot(*this);
	} else {
		// TODO: throw
	}

	tfgg = new generator::tfg(*this, 10);

	sr_ecp_msg->message("ECP TFG loaded");
}

void tfg::mp_2_ecp_next_state_string_handler(void)
{


		if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_TFG) {

			tfgg->Move();
		}


}

}
} // namespace common

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::tfg(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
