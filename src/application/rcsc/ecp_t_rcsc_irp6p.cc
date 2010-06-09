#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"

#include "ecp/irp6p_m/ecp_r_irp6p_m.h"
#include "ecp/common/generator/ecp_g_force.h"
//#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp_t_rcsc_irp6p.h"

namespace mrrocpp {
namespace ecp {
namespace irp6p_m {
namespace task {

// KONSTRUKTORY
rcsc::rcsc(lib::configurator &_config) :
	task(_config) {
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = new irp6p_m::robot(*this);

	gt = new common::generator::transparent(*this);
	nrg = new common::generator::tff_nose_run(*this, 8);
	rgg = new common::generator::tff_rubik_grab(*this, 8);
	gag = new common::generator::tff_gripper_approach(*this, 8);
	rfrg = new common::generator::tff_rubik_face_rotate(*this, 8);
	tig = new common::generator::teach_in(*this);
	befg = new common::generator::bias_edp_force(*this);
	sg = new common::generator::smooth(*this, true);

	go_st = new common::task::ecp_sub_task_gripper_opening(*this);

	sr_ecp_msg->message("ECP loaded");
}

void rcsc::main_task_algorithm(void) {
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");
		//printf("postument: %d\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
		flushall();

		switch ((ecp_mp::task::RCSC_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
		case ecp_mp::task::ECP_GEN_TRANSPARENT:
			gt->throw_kinematics_exceptions
					= (bool) mp_command.ecp_next_state.mp_2_ecp_next_state_variant;
			gt->Move();
			break;
		case ecp_mp::task::ECP_GEN_BIAS_EDP_FORCE:
			befg->Move();
			break;
		case ecp_mp::task::ECP_GEN_TFF_NOSE_RUN:
			nrg->Move();
			break;
		case ecp_mp::task::ECP_GEN_TFF_RUBIK_GRAB:
			switch ((ecp_mp::task::RCSC_RUBIK_GRAB_PHASES) mp_command.ecp_next_state.mp_2_ecp_next_state_variant) {
			case ecp_mp::task::RCSC_RG_FACE_TURN_PHASE_0:
				rgg->configure(0.072, 0.00005, 0, false);
				break;
			case ecp_mp::task::RCSC_RG_FROM_OPEARTOR_PHASE_1:
				rgg->configure(0.057, 0.00005, 0);
				break;
			case ecp_mp::task::RCSC_RG_FROM_OPEARTOR_PHASE_2:
				rgg->configure(0.057, 0.00005, 50);
				break;
			case ecp_mp::task::RCSC_RG_FCHANGE_PHASE_1:
				rgg->configure(0.072, 0.00005, 0, false);
				break;
			case ecp_mp::task::RCSC_RG_FCHANGE_PHASE_2:
				rgg->configure(0.065, 0.00005, 0);
				break;
			case ecp_mp::task::RCSC_RG_FCHANGE_PHASE_3:
				rgg->configure(0.057, 0.00005, 0);
				break;
			case ecp_mp::task::RCSC_RG_FCHANGE_PHASE_4:
				rgg->configure(0.057, 0.00005, 50);
				break;
			default:
				break;
			}
			rgg->Move();
			break;
		case ecp_mp::task::ECP_GEN_TFF_GRIPPER_APPROACH:
			gag->configure(0.005, 150);
			gag->Move();
			break;
		case ecp_mp::task::ECP_GEN_TFF_RUBIK_FACE_ROTATE:
			switch ((ecp_mp::task::RCSC_TURN_ANGLES) mp_command.ecp_next_state.mp_2_ecp_next_state_variant) {
			case ecp_mp::task::RCSC_CCL_90:
				rfrg->configure(-90.0);
				break;
			case ecp_mp::task::RCSC_CL_0:
				rfrg->configure(0.0);
				break;
			case ecp_mp::task::RCSC_CL_90:
				rfrg->configure(90.0);
				break;
			case ecp_mp::task::RCSC_CL_180:
				rfrg->configure(180.0);
				break;
			default:
				break;
			}
			rfrg->Move();
			break;
		case ecp_mp::task::RCSC_GRIPPER_OPENING:
			switch ((ecp_mp::task::RCSC_GRIPPER_OP) mp_command.ecp_next_state.mp_2_ecp_next_state_variant) {
			case ecp_mp::task::RCSC_GO_VAR_1:
				go_st->configure(0.002, 1000);
				go_st->execute();
				break;
			case ecp_mp::task::RCSC_GO_VAR_2:
				go_st->configure(0.02, 1000);
				go_st->execute();
				break;
			default:
				break;
			}
			break;
		case ecp_mp::task::ECP_GEN_TEACH_IN: {
			std::string path(mrrocpp_network_path);
			path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;
			tig->flush_pose_list();
			tig->load_file_with_path(path.c_str());
			//	printf("\nPOSTUMENT ECP_GEN_TEACH_IN :%s\n\n", path1);
			tig->initiate_pose_list();
			tig->Move();
			break;
		}
		case ecp_mp::task::ECP_GEN_SMOOTH: {
			std::string path(mrrocpp_network_path);
			path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;

			switch ((ecp_mp::task::SMOOTH_MOTION_TYPE) mp_command.ecp_next_state.mp_2_ecp_next_state_variant) {
			case ecp_mp::task::RELATIVE:
				sg->set_relative();
				break;
			case ecp_mp::task::ABSOLUTE:
				sg->set_absolute();
				break;
			default:
				break;
			}

			sg->load_file_with_path(path.c_str());
			sg->Move();
			break;
		}
		default:
			break;
		} // end switch

		ecp_termination_notice();
	} //end for
}

}
} // namespace irp6p

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6p_m::task::rcsc(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
