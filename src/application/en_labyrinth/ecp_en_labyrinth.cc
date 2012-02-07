/*
 * ecp_en_labyrinth.cc
 *
 * Author: enatil
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "ecp_en_labyrinth.h"
#include "ecp_mp_g_g_en_labyrinth.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"
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

ecp_en_labyrinth::ecp_en_labyrinth(lib::configurator &_config): common::task::task(_config)
{

	ecp_m_robot = (boost::shared_ptr<robot_t>) new ecp::irp6p_m::robot(*this);
	smooth_generator = new common::generator::newsmooth(*this,lib::ECP_XYZ_ANGLE_AXIS, 6);
	en_generator = new common::generator::g_en_lab(*this);

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

	sr_ecp_msg->message("ECP loaded mm_test");
};

void ecp_en_labyrinth::mp_2_ecp_next_state_string_handler(void)
{
	sr_ecp_msg->message("IN HENDLER");

	ecp_reply.recognized_command[0] = '0';

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH)
	{
//		std::string path(mrrocpp_network_path);
//		path += mp_command.ecp_next_state.sg_buf.get <std::string>();

		std::string position_string = mp_command.ecp_next_state.sg_buf.get <std::string>();

		std::cout << std::endl << "Reading position from MP: " << position_string << std::endl << std::endl;
		std::string buffer;
		std::stringstream position_ss(position_string);

		position_ss >> buffer; // first value in the vector informs about the definition of position values
		if(buffer == "ABSOLUTE_JOIN")
		{
			std::vector<double> position_vec;
			while (position_ss >> buffer)
			{
				double d;
				std::stringstream ss;
				ss << buffer;
				ss >> d;
				position_vec.push_back(d);
			}

			smooth_generator->reset();
			smooth_generator->set_absolute();
			smooth_generator->load_absolute_joint_trajectory_pose(position_vec);
			if(smooth_generator->calculate_interpolate())
			{
				smooth_generator->Move();
			}
			sr_ecp_msg->message("absolute moved");
		}
		else if(buffer == "RELATIVE_EULER")
		{
			std::vector<double> position_vec;
			while (position_ss >> buffer)
			{
				double d;
				std::stringstream ss;
				ss << buffer;
				ss >> d;
				position_vec.push_back(d);
			}

			smooth_generator->reset();
			smooth_generator->set_debug(true);
			smooth_generator->set_absolute();
			smooth_generator->load_relative_angle_axis_trajectory_pose(position_vec);
			//smooth_generator->load_relative_euler_zyz_trajectory_pose(position_vec);
			if(smooth_generator->calculate_interpolate())
			{
				smooth_generator->Move();
			}
			sr_ecp_msg->message("relative moved");
		}
		else
		{
			sr_ecp_msg->message("Cannot define the definition of positions!");
		}

	}
	else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_G_EN_LAB)
	{
		double data_from_mp[2];
		//lib::setValuesInArray(data_from_mp,(char*)mp_command.ecp_next_state.data);
		lib::setValuesInArray(data_from_mp,(char*)mp_command.ecp_next_state.sg_buf.data);
		en_generator->configure(data_from_mp[0],data_from_mp[1]);
		en_generator->Move();
		ecp_reply.recognized_command[0] = en_generator->GEN_REPLY;
	}
}


}
}

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new ecp_en_labyrinth(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp


