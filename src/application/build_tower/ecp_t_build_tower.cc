/*
 * ecp_t_build_tower.cc
 *
 *  Created on: 17-11-2011
 *      Author: spiatek
 */

#include <cmath>
#include <iostream>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"

#include "ecp_t_build_tower.h"

#include "subtask/ecp_st_smooth_file_from_mp.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "subtask/ecp_st_bias_edp_force.h"

#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"
#include "generator/ecp/ecp_g_newsmooth.h"
#include "generator/ecp/ecp_g_multiple_position.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#define BLOCK_WIDTH 3.1
#define BLOCK_HEIGHT 1.9

using namespace mrrocpp::ecp::common::generator;
using namespace logger;

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

build_tower::build_tower(lib::configurator &_config) :
	common::task::task(_config)
{
	if (config.robot_name == lib::irp6p_m::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new irp6p_m::robot(*this);
	} else {
		throw std::runtime_error("Robot not supported");
	}

	log_dbg_enabled = true;

	// utworzenie generatorow
	gtga = new common::generator::tff_gripper_approach(*this, 8);
	sg = new common::generator::newsmooth(*this,lib::ECP_XYZ_ANGLE_AXIS, 6);

	// utworzenie podzadan
	subtask_m[ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE] = new sub_task::bias_edp_force(*this);;
	subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP] = new sub_task::sub_task_smooth_file_from_mp(*this, lib::ECP_JOINT, true);
	subtask_m[ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP] = new sub_task::sub_task_smooth_file_from_mp(*this, lib::ECP_XYZ_ANGLE_AXIS, true);

	sr_ecp_msg->message("ecp BUILD TOWER loaded");
}

void build_tower::mp_2_ecp_next_state_string_handler(void)
{

	if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH) {
		sr_ecp_msg->message("configurate tff_gripper_approach...");
		gtga->configure(0.02, 300, 3);
		gtga->Move();
	}

	else if (mp_2_ecp_next_state_string == ecp_mp::generator::ECP_GEN_NEWSMOOTH) {

		sr_ecp_msg->message("configurate Smooth Generator...");

		int param = (int) mp_command.ecp_next_state.variant;

		char* file_name = (char*) mp_command.ecp_next_state.data;

		sr_ecp_msg->message("after loading param from variant");

		int change_pos[6];

		change_pos[2] = param % 10;
		change_pos[1] = (param % 100 - change_pos[2])/10;
		change_pos[0] = (param % 1000 - change_pos[1])/100;
		change_pos[3] = 0;
		change_pos[4] = 0;
		change_pos[5] = 0;

		sr_ecp_msg->message("change_pos values set");

		if(!(sg->load_trajectory_from_file(file_name))) {
			//TODO: throw exception
			sr_ecp_msg->message("error in loading trajectory from file");
		}

		//sg->print_pose_vector();

		sr_ecp_msg->message("after loading trajectory from file");

		double base_pos[6];

		for (int i = 0; i < 6; i++) {
			base_pos[i] = sg->pose_vector_iterator->coordinates[i];
		}

		sr_ecp_msg->message("after load coordinates");

		lib::setValuesInArray(base_pos, (char*) mp_command.ecp_next_state.data);

		sr_ecp_msg->message("after loading base position");

		int do_move = 0;	//czy zmieniac pozycje
		for(int i = 0; i < 6; ++i) {
			if(abs(base_pos[i]) < 4 && base_pos[i] != 0) {
				do_move = 1;
			}
		}

		if(do_move == 1)
		{
			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates(6);

			/*
			coordinates[0] = 0.966 + change_pos[0]*BLOCK_WIDTH;
			coordinates[1] = 2.320 + change_pos[1]*BLOCK_WIDTH;
			coordinates[2] = 0.088 + change_pos[2]*BLOCK_WIDTH;
			coordinates[3] = 0.217 + change_pos[3]*BLOCK_WIDTH;
			coordinates[4] = 3.107 + change_pos[4]*BLOCK_WIDTH;
			coordinates[5] = 0.073 + change_pos[5]*BLOCK_WIDTH;
			*/

			sr_ecp_msg->message("after reset and set_absoulute");

			coordinates[0] = base_pos[0] + change_pos[0]*BLOCK_WIDTH;
			coordinates[1] = base_pos[1] + change_pos[1]*BLOCK_WIDTH;
			coordinates[2] = base_pos[2] + change_pos[2]*BLOCK_WIDTH;
			coordinates[3] = base_pos[3] + change_pos[3]*BLOCK_WIDTH;
			coordinates[4] = 0.0 + change_pos[4]*BLOCK_WIDTH;
			coordinates[5] = 0.0 + change_pos[5]*BLOCK_WIDTH;

			sr_ecp_msg->message("coordinates ready");

			sg->load_absolute_angle_axis_trajectory_pose(coordinates);

			sr_ecp_msg->message("pose loaded");

			if(sg->calculate_interpolate())
			{
				sg->Move();
			}

			sr_ecp_msg->message("smooth generator configuration end");

		}
		else
		{
			std::cout << "JESTEM TU I NIE WIEM CO ROBIC" << std::endl;
			/*sg->reset();
			sg->set_absolute();
			sg->load_trajectory_from_file(path.c_str());
			if(sg->calculate_interpolate())
			{
				sg->Move();
			}
			*/
		}

	}
}

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new common::task::build_tower(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
