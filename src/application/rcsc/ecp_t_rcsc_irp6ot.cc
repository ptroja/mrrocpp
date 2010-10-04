#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "application/rcsc/ecp_mp_t_rcsc.h"
#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/force/ecp_mp_g_tff_rubik_face_rotate.h"
#include "generator/ecp/force/ecp_mp_g_tff_rubik_grab.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"

//#include "generator/ecp/ecp_g_smooth.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "ecp_t_rcsc_irp6ot.h"
#include "subtask/ecp_st_bias_edp_force.h"
#include "subtask/ecp_st_tff_nose_run.h"

#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/ecp_mp_g_teach_in.h"
#include "generator/ecp/force/ecp_mp_g_weight_measure.h"
#include "subtask/ecp_mp_st_gripper_opening.h"
#include "application/servovision/ecp_mp_g_single_visual_servo_manager.h"

using namespace mrrocpp::ecp::servovision;

namespace mrrocpp {
namespace ecp {
namespace irp6ot_m {
namespace task {

rcsc::rcsc(lib::configurator &_config) :
	task(_config)
{
	// the robot is choose dependendat on the section of configuration file sent as argv[4]
	ecp_m_robot = new irp6ot_m::robot(*this);

	gt = new common::generator::transparent(*this);
	rgg = new common::generator::tff_rubik_grab(*this, 8);
	gag = new common::generator::tff_gripper_approach(*this, 8);
	rfrg = new common::generator::tff_rubik_face_rotate(*this, 8);
	tig = new common::generator::teach_in(*this);

	//sg = new common::generator::smooth(*this, true);
	wmg = new common::generator::weight_measure(*this, 1);

	char fradia_config_section_name[] = { "[fradia_object_follower]" };
	if (config.exists("fradia_task", fradia_config_section_name)) {
		Eigen::Matrix <double, 3, 1> p1, p2;
		p1(0, 0) = 0.6;
		p1(1, 0) = -0.4;
		p1(2, 0) = 0.1;

		p2(0, 0) = 0.95;
		p2(1, 0) = 0.4;
		p2(2, 0) = 0.3;

		//shared_ptr <position_constraint> cube(new cubic_constraint(p1, p2));

		reg = shared_ptr <visual_servo_regulator> (new regulator_p(_config, fradia_config_section_name));
		vs = shared_ptr <visual_servo> (new ib_eih_visual_servo(reg, fradia_config_section_name, _config));
		term_cond = shared_ptr <termination_condition> (new servovision::object_reached_termination_condition(_config, fradia_config_section_name));
		sm
				= shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, fradia_config_section_name, vs));
		//sm->add_position_constraint(cube);
		sm->add_termination_condition(term_cond);
		sm->configure();
	}

	go_st = new common::sub_task::gripper_opening(*this);

	// utworzenie podzadan
	{
		common::sub_task::sub_task* ecpst;
		ecpst = new common::sub_task::bias_edp_force(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = ecpst;
	}

	{
		common::sub_task::tff_nose_run* ecpst;
		ecpst = new common::sub_task::tff_nose_run(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_TFF_NOSE_RUN] = ecpst;
	}

	sr_ecp_msg->message("ecp loaded");
}

rcsc::~rcsc()
{
	delete gt;
	//	delete nrg;
	delete rgg;
	delete gag;
	delete rfrg;
	delete tig;
	//	delete befg;
	//delete sg;
	//delete sg;
	delete wmg;
	delete go_st;
}

void rcsc::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_WEIGHT_MEASURE) {

		wmg->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TRANSPARENT) {
		gt->throw_kinematics_exceptions = (bool) mp_command.ecp_next_state.mp_2_ecp_next_state_variant;
		gt->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_RUBIK_GRAB) {
		switch ((ecp_mp::task::RCSC_RUBIK_GRAB_PHASES) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
		{
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

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH) {
		gag->configure(0.01, 1000);
		gag->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE) {
		switch ((ecp_mp::task::RCSC_TURN_ANGLES) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
		{
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

	} else if (mp_2_ecp_next_state_string == ecp_mp::sub_task::ECP_ST_GRIPPER_OPENING) {
		switch ((ecp_mp::task::RCSC_GRIPPER_OP) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
		{
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

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TEACH_IN) {
		std::string path(mrrocpp_network_path);
		path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;

		tig->flush_pose_list();
		tig->load_file_with_path(path.c_str());
		//		printf("\nTRACK ECP_GEN_TEACH_IN :%s\n\n", path1);
		tig->initiate_pose_list();

		tig->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {
		std::string path(mrrocpp_network_path);
		path += mp_command.ecp_next_state.mp_2_ecp_next_state_string;

		switch ((ecp_mp::task::SMOOTH_MOTION_TYPE) mp_command.ecp_next_state.mp_2_ecp_next_state_variant)
		{
			case ecp_mp::task::RELATIVE:
				//sg->set_relative();
				break;
			case ecp_mp::task::ABSOLUTE:
				//sg->set_absolute();
				break;
			default:
				break;
		}

		//sg->load_file_with_path(path.c_str());
		//sg->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_IB_EIH) {
		sm->Move();

	}

}

}
} // namespace irp6ot

namespace common {
namespace task {

task* return_created_ecp_task(lib::configurator &_config)
{
	return new irp6ot_m::task::rcsc(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

