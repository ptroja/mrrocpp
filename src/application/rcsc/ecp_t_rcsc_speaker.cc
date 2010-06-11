// ------------------------------------------------------------------------
//   task/ecp_t_tran.cc - przezroczyste wersja dla dowolnego z robotow
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2006
// ------------------------------------------------------------------------

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp/speaker/ecp_r_speaker.h"
#include "ecp_t_rcsc_speaker.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"

namespace mrrocpp {
namespace ecp {
namespace speaker {
namespace task {

// KONSTRUKTORY
rcsc::rcsc(lib::configurator &_config) :
	task(_config)
{
	ecp_m_robot = new robot(*this);

	gt = new common::generator::transparent(*this);
	speak = new generator::speaking(*this, 8);

	sr_ecp_msg->message("ECP loaded");
}

void rcsc::main_task_algorithm(void)
{
	for (;;) {

		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");

		if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_TRANSPARENT) {
			gt->Move();

		} else if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_SPEAK) {
			speak->configure(mp_command.ecp_next_state.mp_2_ecp_next_state_string);
			speak->Move();

		}

	}
}

}
} // namespace speaker

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new speaker::task::rcsc(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

