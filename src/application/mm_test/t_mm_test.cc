/*
 * t_mm_test.cc
 *
 *  Created on: Apr 13, 2010
 *      Author: mmichnie
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>


//#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
//#include "robot/irp6p_m/ecp_r_irp6p_m.h"
#include "t_mm_test.h"
//#include "base/lib/datastr.h"
#include "ecp_mp_g_g_mm_test.h"
//#include "../defines.h"
#include "generator/ecp/ecp_g_newsmooth.h"

#include "robot/irp6p_m/ecp_r_irp6p_m.h"

//#include "../ecp_mp_g_visual_servo_tester.h"

#include "../edge_follow/ecp_st_edge_follow.h"
#include "subtask/ecp_st_bias_edp_force.h"
#include "subtask/ecp_st_tff_nose_run.h"

#include "subtask/ecp_mp_st_bias_edp_force.h"

using namespace mrrocpp::ecp::common::generator;
using namespace logger;


namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

//Constructors
mm_test::mm_test(lib::configurator &_config): common::task::task(_config)
{
	/*
	if (config.section_name == lib::irp6ot_m::ECP_SECTION) {
			//ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6ot_m::robot(*this);
			//sg = new common::generator::newsmooth(*this,lib::ECP_JOINT, 7);
		} else if (config.section_name == lib::irp6p_m::ECP_SECTION) {
			ecp_m_robot = (boost::shared_ptr<robot_t>) new irp6p_m::robot(*this);
			sg = new common::generator::newsmooth(*this,lib::ECP_XYZ_ANGLE_AXIS, 6);
		} else {
			// TODO: throw, robot unsupported
			return;
		}
	*/

	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);
	sg = new common::generator::newsmooth(*this,lib::ECP_XYZ_ANGLE_AXIS, 6);

	gen = new common::generator::g_mm_test(*this);

	/***/
	// utworzenie podzadan
	{
		sub_task::sub_task* ecpst;
		ecpst = new sub_task::edge_follow(*this);
		subtask_m[ecp_mp::sub_task::EDGE_FOLLOW] = ecpst;

		ecpst = new sub_task::bias_edp_force(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = ecpst;
	}

	{
		sub_task::tff_nose_run* ecpst;
		ecpst = new sub_task::tff_nose_run(*this);
		subtask_m[ecp_mp::sub_task::ECP_ST_TFF_NOSE_RUN] = ecpst;
		ecpst->nrg->configure_pulse_check(true);
	}

	sr_ecp_msg->message("ecp edge_follow_MR loaded");
	/***/





	char config_section_name[] = { "[object_follower_ib]" };

	log_dbg_enabled = true;
	log_enabled = true;

	shared_ptr <position_constraint> cube(new cubic_constraint(config, config_section_name));

	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 1\n");
	reg = shared_ptr <visual_servo_regulator> (new regulator_p(config, config_section_name));

	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 2\n");

	boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor> ds = boost::shared_ptr <mrrocpp::ecp_mp::sensor::discode::discode_sensor>(new mrrocpp::ecp_mp::sensor::discode::discode_sensor(config, config_section_name));
	vs = shared_ptr <visual_servo> (new ib_eih_visual_servo(reg, ds, config_section_name, config));

	term_cond = shared_ptr <termination_condition> (new object_reached_termination_condition(config, config_section_name));

	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 3\n");
	sm = shared_ptr <single_visual_servo_manager> (new single_visual_servo_manager(*this, config_section_name, vs));

	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 4\n");
	sm->add_position_constraint(cube);

	sm->add_termination_condition(term_cond);
	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 5\n");

	sm->configure();
	log_dbg("ecp_t_objectfollower_ib_eih::ecp_t_objectfollower_ib_eih(): 6\n");



	//my_generator = new generator::g_mm_test(*this);
	sr_ecp_msg->message("ECP loaded mm_test");
};

void mm_test::move_down(double mm)
{
	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose * actTrajectory = new ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose();
	actTrajectory->arm_type = lib::ECP_XYZ_ANGLE_AXIS;
	for (int i=0;i<6;i++)
	{
		actTrajectory->v.push_back(0.02);
		actTrajectory->a.push_back(0.03);
	}
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(mm);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	sg->load_relative_pose((*actTrajectory));
}
void mm_test::move_right(double mm)
{
	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose * actTrajectory = new ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose();
	actTrajectory->arm_type = lib::ECP_XYZ_ANGLE_AXIS;
	for (int i=0;i<6;i++)
	{
		actTrajectory->v.push_back(0.02);
		actTrajectory->a.push_back(0.03);
	}
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(mm);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	sg->load_relative_pose((*actTrajectory));
}

void mm_test::move_back(double mm)
{
	ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose * actTrajectory = new ecp_mp::common::trajectory_pose::bang_bang_trajectory_pose();
	actTrajectory->arm_type = lib::ECP_XYZ_ANGLE_AXIS;
	for (int i=0;i<6;i++)
	{
		actTrajectory->v.push_back(0.02);
		actTrajectory->a.push_back(0.03);
	}
	actTrajectory->coordinates.push_back(mm);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	actTrajectory->coordinates.push_back(0);
	sg->load_relative_pose((*actTrajectory));
}

void mm_test::mp_2_ecp_next_state_string_handler(void)
{
	sr_ecp_msg->message("IN HENDLER");

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH)
	{
		//get_next_state();
		sr_ecp_msg->message("rozkaz odebrany");
		std::string path(mrrocpp_network_path);
		path += (char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string;

		if(((char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string)[0]<= '9' && ((char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string)[0]>= '0')
		{
			double t[2];
			lib::setValuesInArray(t,(char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string);

			if(t[0] < 1.5)
			{
				move_down(t[1]);
			}
			else if(t[0] > 2.5)
			{
				move_back(t[1]);
			}
			else
			{
				move_right(t[1]);
			}
		}
		else
		{
			sg->load_trajectory_from_file(path.c_str());
		}
		sg->calculate_interpolate();
		sg->Move();
		sr_ecp_msg->message("moved");

		//ecp_termination_notice();
		//sr_ecp_msg->message("noticed");

	}

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_G_MM_TEST)
	{
		gen->Move();
	}

	sr_ecp_msg->message("HENDLER END");

}


/*
void mm_test::main_task_algorithm(void ) {

	sr_ecp_msg->message("max's test ready");
*/
	//get_next_state();
	//sr_ecp_msg->message("rozkaz odebrany");
	//std::string path(mrrocpp_network_path);
	//path += (char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string;

	//sg->load_trajectory_from_file(path.c_str());
	//sg->calculate_interpolate();
	//sg->Move();
	//sr_ecp_msg->message("moved");

	//get_next_state();
	//sr_ecp_msg->message("servo move start");
	//sm->Move();
	//sr_ecp_msg->message("servo move end");

	//while (1)
	//{
	//	get_next_state();
	//	if (mp_2_ecp_next_state_string == mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST) {
//			sm->Move();
	//	} else {
	//		log("ecp_t_objectfollower_ib_eih::main_task_algorithm(void) mp_2_ecp_next_state_string: \"%s\"\n", mp_2_ecp_next_state_string.c_str());
	//	}
	//}
/*
	for(;;)
	{
		get_next_state();
		sr_ecp_msg->message("rozkaz odebrany");
		std::string path(mrrocpp_network_path);
		path += (char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string;

		if(((char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string)[0]<= '9' && ((char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string)[0]>= '0')
		{
			double t[2];
			lib::setValuesInArray(t,(char*)mp_command.ecp_next_state.mp_2_ecp_next_state_string);

			if(t[0] < 1.5)
			{
				move_down(t[1]);
			}
			else if(t[0] > 2.5)
			{
				move_back(t[1]);
			}
			else
			{
				move_right(t[1]);
			}
		}
		else
		{
			sg->load_trajectory_from_file(path.c_str());
		}
		sg->calculate_interpolate();
		sg->Move();
		sr_ecp_msg->message("moved");

		ecp_termination_notice();
	sr_ecp_msg->message("noticed");
	}
	*/
/*
	ecp_termination_notice();
};
*/
}
} // namespace irp6ot

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new mm_test(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


