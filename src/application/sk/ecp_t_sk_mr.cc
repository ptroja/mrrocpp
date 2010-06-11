#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp_t_sk_mr.h"

#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp/common/generator/ecp_g_sleep.h"

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/irp6p_m/ecp_r_irp6p_m.h"

#include "ecp_t_sk_mr.h"
#include "ecp_st_edge_follow.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

// KONSTRUKTORY
sk_mr::sk_mr(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	if (config.section_name == ECP_IRP6OT_M_SECTION) {
		ecp_m_robot = new irp6ot_m::robot(*this);
	} else if (config.section_name == ECP_IRP6P_M_SECTION) {
		ecp_m_robot = new irp6p_m::robot(*this);
	} else {
		// TODO: throw
	}

	nrg = new generator::tff_nose_run(*this, 8);
	nrg->configure_pulse_check(true);
	yefg = new generator::y_edge_follow_force(*this, 8);
	befg = new generator::bias_edp_force(*this);

	// utworzenie podzadania
	ecp_sub_task* ecpst;
	ecpst = new ecp_sub_task_edge_follow(*this);
	subtask_m[ecp_mp::task::ECP_ST_EDGE_FOLLOW] = ecpst;

	sr_ecp_msg->message("ECP SK_MR loaded");
}

void sk_mr::main_task_algorithm(void)
{
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");
		//printf("postument: %d\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
		//flushall();

		subtasks_conditional_execution();

		if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE) {
			befg->Move();
		} else if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_TFF_NOSE_RUN) {
			nrg->Move();
		}
		/*else if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_EDGE_FOLLOW_FORCE) {
		 yefg->Move();
		 }*/

		ecp_termination_notice();
	} //end for
}

}
} // namespace common

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::sk_mr(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
