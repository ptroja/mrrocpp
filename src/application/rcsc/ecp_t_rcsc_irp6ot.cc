#include <stdio.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "ecp_mp/task/ecp_mp_t_rcsc.h"

#include "ecp/irp6ot_m/ecp_r_irp6ot_m.h"
#include "ecp/common/generator/ecp_g_force.h"
//#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp/common/generator/ecp_g_smooth.h"
#include "ecp_t_rcsc_irp6ot.h"

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

rcsc::rcsc(lib::configurator &_config) :
	task(_config) {
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = new irp6ot_m::robot(*this);

	gt = new common::generator::transparent(*this);
	nrg = new common::generator::tff_nose_run(*this, 8);
	rgg = new common::generator::tff_rubik_grab(*this, 8);
	gag = new common::generator::tff_gripper_approach(*this, 8);
	rfrg = new common::generator::tff_rubik_face_rotate(*this, 8);
	tig = new common::generator::teach_in(*this);
	befg = new common::generator::bias_edp_force(*this);
	sg = new common::generator::smooth(*this, true);
	wmg = new common::generator::weight_meassure(*this, 1);

	char fradia_config_section_name[] = { "[fradia_object_follower]" };
	if (config.exists("fradia_task", fradia_config_section_name)) {
		Eigen::Matrix<double, 3, 1> p1, p2;
		p1(0, 0) = 0.6;
		p1(1, 0) = -0.4;
		p1(2, 0) = 0.1;

		p2(0, 0) = 0.95;
		p2(1, 0) = 0.4;
		p2(2, 0) = 0.3;

		shared_ptr<position_constraint> cube(new cubic_constraint(p1, p2));

		reg = shared_ptr<visual_servo_regulator> (new regulator_p(_config,
				fradia_config_section_name));
		vs = shared_ptr<visual_servo> (new ib_eih_visual_servo(reg,
				fradia_config_section_name, _config));
		term_cond = shared_ptr<termination_condition> (
				new object_reached_termination_condition(0.005, 0.005, 50));
		sm = shared_ptr<simple_visual_servo_manager> (
				new simple_visual_servo_manager(*this,
						fradia_config_section_name, vs));
		sm->add_position_constraint(cube);
		sm->add_termination_condition(term_cond);
		sm->configure();
	}

	go_st = new common::task::ecp_sub_task_gripper_opening(*this);

	sr_ecp_msg->message("ECP loaded");
}

rcsc::~rcsc() {
	delete gt;
	delete nrg;
	delete rgg;
	delete gag;
	delete rfrg;
	delete tig;
	delete befg;
	//delete sg;
	delete sg;
	delete wmg;
	delete go_st;
}

void rcsc::main_task_algorithm(void) {
	for (;;) {
		sr_ecp_msg->message("Waiting for MP order");

		get_next_state();

		sr_ecp_msg->message("Order received");
		//printf("track: %d\n", mp_command.ecp_next_state.mp_2_ecp_next_state);
		flushall();
		switch ((ecp_mp::task::RCSC_ECP_STATES) mp_command.ecp_next_state.mp_2_ecp_next_state) {
		case ecp_mp::task::ECP_WEIGHT_MEASURE_GENERATOR:
			wmg->Move();
			break;
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
			gag->configure(0.01, 1000);
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
			//		printf("\nTRACK ECP_GEN_TEACH_IN :%s\n\n", path1);
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
		case ecp_mp::task::ECP_GEN_IB_EIH: {

			sm->Move();
			break;
		}

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

task* return_created_ecp_task(lib::configurator &_config) {
	return new irp6ot_m::task::rcsc(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

