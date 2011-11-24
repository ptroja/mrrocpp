/*
 * mp_t_build_tower.cc
 *
 *  Created on: 17-11-2011
 *      Author: spiatek
 */

#include <fstream>
#include <iostream>

#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"

#include "base/lib/mrmath/mrmath.h"
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "generator/ecp/force/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/ecp_mp_g_newsmooth.h"

#include "subtask/ecp_mp_st_smooth_file_from_mp.h"
#include "subtask/ecp_mp_st_bias_edp_force.h"

#include "mp_t_build_tower.h"

#define COORD_N 3

typedef list<BlockPosition> block_position_list;

using namespace std;

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new build_tower(_config);
}


void build_tower::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
}

build_tower::build_tower(lib::configurator &_config) :
	task(_config)
{
}

block_position_list build_tower::get_list_from_file(const char* file_name)
{
	sr_ecp_msg->message("Get List From File Start");

	int number_of_blocks;
	int number_of_coordinates = COORD_N;
	char color[8];
	const char* col;
	std::vector <int> positions(number_of_coordinates);
	BlockPosition *block_pos;

	block_position_list pb_lst;

	std::ifstream file_stream(file_name);

	if (!file_stream.good()) {
		//TODO: add exception - file not exists
		//throw MP_error(lib::NON_FATAL_ERROR, NON_EXISTENT_FILE);
		sr_ecp_msg->message("File not exists");
		return pb_lst;
	}

	sr_ecp_msg->message("File opened");

	if (!(file_stream >> number_of_blocks)) {
		//TODO: add exception - read from file
		//throw MP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
		sr_ecp_msg->message("Can't read from file");
		return pb_lst;
	}

	for(int i = 0; i < number_of_blocks; ++i) {

		if(!(file_stream >> color)) {
			//TODO: add exception - read from file
			//throw MP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
			sr_ecp_msg->message("Can't read color info");
			return pb_lst;
		}

		file_stream.ignore(std::numeric_limits<std::streamsize>::max(), ' ');

		if(!strcmp(color, "BLUE")) {
			col = "blue";
		}
		else if(!strcmp(color, "RED")) {
			col = "red";
		}
		else if(!strcmp(color, "GREEN")) {
			col = "green";
		}
		else if(!strcmp(color, "YELLOW")) {
			col = "yellow";
		}
		else {
			//TODO: add exception - bad color
			//throw MP_error(lib::NON_FATAL_ERROR, NON_CONFIG_FILE);
			return pb_lst;
		}

		for(int j = 0; j < 3; j++) {
			if(!(file_stream >> positions[j])) {
				sr_ecp_msg->message("Can't read coordinate");
				//TODO: add exception - bad file format
				return pb_lst;
			}
		}

		for(int j = 0; j < number_of_coordinates; ++j) {
			cout << positions[j] << " ";
		}

		block_pos = new BlockPosition(col, positions);
		pb_lst.push_back(*block_pos);

		sr_ecp_msg->message("Get from file end");
	}

	return pb_lst;
}

block_position_list build_tower::create_plan(block_position_list l)
{
	sr_ecp_msg->message("Creating plan");

	return l;
}

void build_tower::main_task_algorithm(void)
{
	sr_ecp_msg->message("Build Tower MP Start");

	block_position_list list_from_file = get_list_from_file("../../src/application/build_tower/con/structure.con");
	planned_list = create_plan(list_from_file);

	for(block_position_list::iterator i = planned_list.begin(); i != planned_list.end(); ++i) {

		sr_ecp_msg->message("Inside main loop");

		present_color = (*i).getColor();
		present_position = (*i).getPosition();

		cout << "color: " << present_color << endl;
		cout << "position: " << endl;
		for(int j = 0; j < 3; ++j) {
			cout << present_position[j] << " ";
		}
		cout << endl;

		sr_ecp_msg->message("Waiting for blue block...");
		set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/build_tower/trjs/front_position.trj", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		//Zerowanie czujników
		sr_ecp_msg->message("Postument Bias");
		set_next_ecp_state(ecp_mp::sub_task::ECP_ST_BIAS_EDP_FORCE, 5, "", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		wait_ms(8000);

		int param = 100*present_position[0] + 10*present_position[1] + present_position[2];

		//TODO: przekazywanie pozycji do ecp
		sr_ecp_msg->message("Reaching the tower place...");		//zakladam, ze pozycja jest niezmienna
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, param, "../../src/application/build_tower/trjs/pos_build_start.trj", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		wait_ms(4000);

		sr_ecp_msg->message("Force approach...");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, 5, "", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		wait_ms(4000);

		//3.1, 1.9 - wymiary klocka

		sr_ecp_msg->message("Raising up...");
		set_next_ecp_state(ecp_mp::sub_task::ECP_ST_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/build_tower/trjs/build.trj", 0, lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, 1, lib::irp6p_m::ROBOT_NAME.c_str());

		wait_ms(4000);

	}

	sr_ecp_msg->message("Build Tower END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
