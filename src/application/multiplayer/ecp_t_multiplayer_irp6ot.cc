#include <cstdio>
#include <cstring>

#include "base/lib/sr/srlib.h"
#include "ecp_mp_t_multiplayer.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "generator/ecp/ecp_g_newsmooth.h"

#include "ecp_t_multiplayer_irp6ot.h"
#include "subtask/ecp_st_bias_edp_force.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "subtask/ecp_mp_st_gripper_opening.h"

#include "base/ecp/ecp_task.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/force/ecp_mp_g_weight_measure.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot {
namespace task {

multiplayer::multiplayer(lib::configurator &_config) :
	task(_config)
{
	ecp_m_robot = new irp6ot_m::robot(*this);

	//powolanie generatorow

	//sg = new common::generator::smooth(*this, true);
	wmg = new common::generator::weight_measure(*this, -0.3, 2);
	gt = new common::generator::transparent(*this);

	go_st = new common::sub_task::gripper_opening(*this);

	rgg = new common::generator::tff_rubik_grab(*this, 8);

	// utworzenie podzadan
	{
		common::sub_task::sub_task* ecpst;

		ecpst = new common::sub_task::bias_edp_force(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = ecpst;
	}

	sr_ecp_msg->message("ecp loaded");
}

void multiplayer::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_WEIGHT_MEASURE) {

		wmg->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TRANSPARENT) {
		gt->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::sub_task::ECP_ST_GRIPPER_OPENING) {
		switch ((ecp_mp::task::MULTIPLAYER_GRIPPER_OP) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
		{
			case ecp_mp::task::MULTIPLAYER_GO_VAR_1:
				go_st->configure(0.002, 1000);
				go_st->execute();
				break;
			case ecp_mp::task::MULTIPLAYER_GO_VAR_2:
				go_st->configure(0.02, 1000);
				go_st->execute();
				break;
			default:
				break;
		}
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {
		std::string path(mrrocpp_network_path);
		path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;
		//sg->load_file_with_path(path.c_str());
		//printf("\nTRACK ECP_GEN_SMOOTH :%s\n\n", path1);
		//sg->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_TAKE_FROM_ROVER) {
		sr_ecp_msg->message("MOVE");
		// takeg->Move();
		sr_ecp_msg->message("STOP MOVE");
	} else if (mp_2_ecp_next_state_string == ecp_mp::task::ECP_GEN_GRAB_FROM_ROVER) {
		sr_ecp_msg->message("GRAB");
		rgg->configure(0.057, 0.00005, 0);
		rgg->Move();
		sr_ecp_msg->message("STOP GRAB");
	}

}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot::task::multiplayer(_config);
}
}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


