// ------------------------------------------------------------------------
//   ecp_t_pouring_irp6ot.cc - zadanie przelewania, ECP dla IRP6_ON_TRACK
//
//                     EFFECTOR CONTROL PROCESS (lib::ECP) - main()
//
// Ostatnia modyfikacja: 2008
// ------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/ecp_mp_t_pouring.h"

#include "ecp/irp6_on_track/ecp_r_irp6ot.h"
#include "ecp/common/ecp_g_smooth.h"
#include "ecp/common/ecp_st_go.h"
#include "ecp/irp6_on_track/ecp_t_pouring_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

// KONSTRUKTORY
pouring::pouring(lib::configurator &_config) : task(_config)
{
    sg = NULL;
    tcg = NULL;
}


// methods for ECP template to redefine in concrete classes
void pouring::task_initialization(void)
{
    // the robot is choose dependendat on the section of configuration file sent as argv[4]
    ecp_m_robot = new robot (*this);

    sg = new common::generator::smooth (*this, true);
    tcg = new common::generator::tool_change(*this, true);

    go_st = new common::task::ecp_sub_task_gripper_opening(*this);

    sr_ecp_msg->message("ECP loaded");
}


void pouring::main_task_algorithm(void)
{
	for(;;)
	{
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state ();

		sr_ecp_msg->message("Order received");

		switch ( (ecp_mp::task::POURING_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state)
		{
		case ecp_mp::task::ECP_GEN_SMOOTH:
		{
			std::string path(mrrocpp_network_path);
			path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;

			sg->load_file_with_path (path.c_str());
			//printf("\nON_TRACK ECP_GEN_SMOOTH :%s\n\n", path1);
			//printf("OT po delete\n");
			sg->Move();
			//printf("OT po move\n");
			break;
		}
		case ecp_mp::task::ECP_GEN_POURING:
			tcg->set_tool_parameters(-0.18, 0.0, 0.25);
			tcg->Move();
			break;
		case ecp_mp::task::ECP_END_POURING:
			tcg->set_tool_parameters(0.0, 0.0, 0.25);
			tcg->Move();
			break;
		case ecp_mp::task::GRIP:
			go_st->configure(-0.03, 1000);
			go_st->execute();
			break;
		case ecp_mp::task::LET_GO:
			go_st->configure(0.03, 1000);
			go_st->execute();
			break;
		default:
			break;
		}
		ecp_termination_notice();

	} //end for
}

}
} // namespace irp6ot

namespace common {
namespace task {
task* return_created_ecp_task (lib::configurator &_config)
{
	return new irp6ot::task::pouring(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

