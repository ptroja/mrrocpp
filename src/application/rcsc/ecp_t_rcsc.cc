#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"
#include "application/rcsc/ecp_mp_t_rcsc.h"
#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/force/ecp_mp_g_tff_rubik_face_rotate.h"

#include "ecp_t_rcsc.h"
#include "generator/ecp/force/ecp_g_bias_edp_force.h"
#include "generator/ecp/force/ecp_g_tff_nose_run.h"

#include "generator/ecp/ecp_mp_g_transparent.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/ecp_mp_g_teach_in.h"
#include "generator/ecp/force/ecp_mp_g_weight_measure.h"

/*
 #include "application/servovision/ecp_mp_g_single_visual_servo_manager.h"

 using namespace mrrocpp::ecp::servovision;
 */
namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

rcsc::rcsc(lib::configurator &_config) :
		common::task::task(_config)
{

	// the robot is choose dependently on the section of configuration file sent as argv[4]
	if (config.robot_name == lib::irp6ot_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6ot_m::robot(*this);
	} else if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		// TODO: throw
		throw std::runtime_error("Robot not supported");
	}

	gt = new generator::transparent(*this);
	gag = new generator::tff_gripper_approach(*this, 8);
	rfrg = new generator::tff_rubik_face_rotate(*this, 8);
	tig = new generator::teach_in(*this);

	sg = new generator::newsmooth(*this, lib::ECP_JOINT, ecp_m_robot->number_of_servos);
	sg->set_debug(true);
	sgaa = new generator::newsmooth(*this, lib::ECP_XYZ_ANGLE_AXIS, 6);
	sgaa->set_debug(true);

	register_generator(new generator::bias_edp_force(*this));

	{
		common::generator::tff_nose_run *ecp_gen = new common::generator::tff_nose_run(*this, 8);
		register_generator(ecp_gen);
	}

	register_generator(new generator::weight_measure(*this, 1));

	sr_ecp_msg->message("ecp loaded");
}

rcsc::~rcsc()
{
	delete gt;
	//	delete nrg;

	delete gag;
	delete rfrg;
	delete tig;
	//	delete befg;
	delete sg;
	delete sgaa;
}

void rcsc::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TRANSPARENT) {
		gt->throw_kinematics_exceptions = (bool) mp_command.ecp_next_state.variant;
		gt->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH) {
		gag->configure(0.01, 1000, 3);
		gag->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_RUBIK_FACE_ROTATE) {
		switch ((ecp_mp::task::RCSC_TURN_ANGLES) mp_command.ecp_next_state.variant)
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

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TEACH_IN) {
		std::string path(mrrocpp_network_path);
		path += (char*) mp_command.ecp_next_state.sg_buf.data;

		tig->flush_pose_list();
		tig->load_file_with_path(path);
		//		printf("\nTRACK ECP_GEN_TEACH_IN :%s\n\n", path1);
		tig->initiate_pose_list();

		tig->Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH
			|| mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH_JOINT) {
		std::string path(mrrocpp_network_path);
		path += mp_command.ecp_next_state.sg_buf.get <std::string>();

		switch ((lib::MOTION_TYPE) mp_command.ecp_next_state.variant)
		{
			case lib::RELATIVE:
				sg->set_relative();
				break;
			case lib::ABSOLUTE:
				sg->set_absolute();
				break;
			default:
				break;
		}
		sg->reset();
		sg->load_trajectory_from_file(path.c_str());
		sg->calculate_interpolate();
		sg->Move();
	} else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH_ANGLE_AXIS) {
		std::string path(mrrocpp_network_path);
		path += mp_command.ecp_next_state.sg_buf.get <std::string>();

		switch ((lib::MOTION_TYPE) mp_command.ecp_next_state.variant)
		{
			case lib::RELATIVE:
				sgaa->set_relative();
				break;
			case lib::ABSOLUTE:
				sgaa->set_absolute();
				break;
			default:
				break;
		}
		sgaa->reset();
		sgaa->load_trajectory_from_file(path.c_str());
		sgaa->calculate_interpolate();
		sgaa->Move();
	}

}

}
} // namespace irp6ot

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::rcsc(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

