/*
 * ecp_t_build_tower.cc
 *
 *  Created on: 17-11-2011
 *      Author: spiatek
 */

#include <cmath>

#include "base/lib/configurator.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"

#include "ecp_t_build_tower.h"

#include "subtask/ecp_st_smooth_file_from_mp.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"
#include "subtask/ecp_st_bias_edp_force.h"

#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"

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

		double change_pos[6];
		lib::setValuesInArray(change_pos,(char*)mp_command.ecp_next_state.data);

		int do_move = 0;	//czy zmieniac pozycje
		for(int i = 0; i < 6; ++i) {
			if(abs(change_pos[i]) < 4 && change_pos[i] != 0) {
				do_move = 1;
			}
		}

		if(do_move == 1)
		{
			sg->reset();
			sg->set_absolute();

			std::vector <double> coordinates(6);

			//d - wektor przesylany do funkcji
			//wspolrzedne docelowo beda wyliczane w inny sposob

			coordinates[0] = 0.966 + change_pos[0]*BLOCK_WIDTH;
			coordinates[1] = 2.320 + change_pos[1]*BLOCK_WIDTH;
			coordinates[2] = 0.088 + change_pos[2]*BLOCK_WIDTH;
			coordinates[3] = 0.217 + change_pos[3]*BLOCK_WIDTH;
			coordinates[4] = 3.107 + change_pos[4]*BLOCK_WIDTH;
			coordinates[5] = 0.073 + change_pos[5]*BLOCK_WIDTH;

			sg->load_absolute_angle_axis_trajectory_pose(coordinates);

			if(sg->calculate_interpolate())
			{
				sg->Move();
			}
		}
		else
		{
			cout << "JESTEM TU I NIE WIEM CO ROBIC" << endl;
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
