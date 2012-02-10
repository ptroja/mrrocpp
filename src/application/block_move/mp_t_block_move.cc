#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include "base/mp/mp_task.h"
#include "base/mp/mp_robot.h"

#include "mp_t_block_move.h"
#include "BlockPosition.h"

#include "base/lib/mrmath/mrmath.h"
#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"
#include "base/lib/configurator.h"

//#include "robot/irp6_tfg/dp_tfg.h"
//#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
//#include "robot/irp6p_tfg/const_irp6p_tfg.h"
#include "robot/irp6p_m/const_irp6p_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"
#include "generator/ecp/newsmooth/ecp_mp_g_newsmooth.h"

#include "generator/ecp/smooth_file_from_mp/ecp_mp_g_smooth_file_from_mp.h"

#include "generator/ecp/tff_gripper_approach/ecp_mp_g_tff_gripper_approach.h"

//#include "subtask/ecp_mp_st_gripper_opening.h"
#include "generator/ecp/bias_edp_force/ecp_mp_g_bias_edp_force.h"

#include "../visual_servoing/visual_servoing.h"
#include "../visual_servoing_demo/ecp_mp_g_visual_servo_tester.h"

#define WIDTH 4
#define BOARD_COLOR 5
#define BLOCK_SIZE 4
#define COORD_N 3
#define BLOCK_REACHING 0
#define BUILDING 1
#define COUNT_SERVO_TRY_1 5
#define COUNT_SERVO_TRY_2 5

typedef list <BlockPosition> block_position_list;

using namespace std;

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new block_move(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void block_move::create_robots()
{
	ACTIVATE_MP_ROBOT(irp6p_m);
}

block_move::block_move(lib::configurator &_config) :
		task(_config)
{
}

block_position_list block_move::get_list_from_file(const char* file_name)
{
	sr_ecp_msg->message("Get List From File Start");

	int number_of_blocks;
	int number_of_coordinates = COORD_N;
	char color[8];
	int col;
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

	for (int i = 0; i < number_of_blocks; ++i) {

		if (!(file_stream >> color)) {
			//TODO: add exception - read from file
			//throw MP_error(lib::NON_FATAL_ERROR, READ_FILE_ERROR);
			sr_ecp_msg->message("Can't read color info");
			return pb_lst;
		}

		file_stream.ignore(std::numeric_limits <std::streamsize>::max(), ' ');

		if (!strcmp(color, "BLUE")) {
			col = 1;
		} else if (!strcmp(color, "RED")) {
			col = 2;
		} else if (!strcmp(color, "GREEN")) {
			col = 3;
		} else if (!strcmp(color, "YELLOW")) {
			col = 4;
		} else {
			//TODO: add exception - bad color
			//throw MP_error(lib::NON_FATAL_ERROR, NON_CONFIG_FILE);
			return pb_lst;
		}

		for (int j = 0; j < 3; j++) {
			if (!(file_stream >> positions[j])) {
				sr_ecp_msg->message("Can't read coordinate");
				//TODO: add exception - bad file format
				return pb_lst;
			}
		}

		block_pos = new BlockPosition(col, positions);
		pb_lst.push_back(*block_pos);

		sr_ecp_msg->message("Get from file end");
	}

	return pb_lst;
}

block_position_list block_move::create_plan(block_position_list l)
{
	sr_ecp_msg->message("Creating plan");

	l.sort();

	cout << "Plan:" << endl;
	for(block_position_list::iterator it = l.begin(); it != l.end(); ++it) {
		(*it).print();
	}

	block_position_list plan;

#ifdef USE_GECODE

	BlockPlanner* bp = new BlockPlanner(WIDTH, l.size(), l);	//definition of CSP

	DFS<BlockPlanner> e(bp);									//searching for solution

	bp->print();												//printing solution
	plan = bp->getPlan();										//getting a solution
	delete bp;

	sr_ecp_msg->message("Creating plan end");

	return plan;

#else

	return l;

#endif

}

void block_move::main_task_algorithm(void)
{
	sr_ecp_msg->message("Block Move MP Start");

	block_position_list list_from_file = get_list_from_file("../../src/application/block_move/con/structure.con");
	planned_list = create_plan(list_from_file);

	int board_localization = config.value <int>("board_localization", "[mp_block_move]");
	int count_servo_try_1 = config.value <int>("count_servo_try_1", "[mp_block_move]");

	if (board_localization == 1) {

		for (int h = 0; h < count_servo_try_1; ++h) {

			sr_ecp_msg->message("Reaching building place...");

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/pos_build_start.trj", lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

			wait_ms(1000);

			sr_ecp_msg->message("Board localization - servovision");

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST, BOARD_COLOR, "", lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

			if (robot_m[lib::irp6p_m::ROBOT_NAME]->ecp_reply_package.variant == 1) {
				sr_ecp_msg->message("Board localized");
				break;
			}

			sr_ecp_msg->message("Board not localized!!!");
		}
	}

	for (block_position_list::iterator i = planned_list.begin(); i != planned_list.end(); ++i) {

		sr_ecp_msg->message("Inside main loop");

		present_color = (*i).getColor();
		present_position = (*i).getPosition();

		(*i).print();

		sr_ecp_msg->message("Start position");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/pos_search_area_start.trj", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		int block_localization = config.value <int>("block_localization", "[mp_block_move]");
		int count_servo_try_2 = config.value <int>("count_servo_try_2", "[mp_block_move]");

		if (block_localization == 1) {

			for (int h = 0; h < count_servo_try_2; ++h) {

				wait_ms(1000);

				sr_ecp_msg->message("Start position");

				set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/pos_search_area_start.trj", lib::irp6p_m::ROBOT_NAME);
				wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

				wait_ms(1000);

				sr_ecp_msg->message("Block localization - servovision");

				set_next_ecp_state(ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST, present_color, "", lib::irp6p_m::ROBOT_NAME);
				wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

				if (robot_m[lib::irp6p_m::ROBOT_NAME]->ecp_reply_package.variant == 1) {
					sr_ecp_msg->message("Block localized");
					break;
				}

				sr_ecp_msg->message("Block not localized!!!");
			}

			wait_ms(1000);

			sr_ecp_msg->message("Force approach");

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.03, 800, 3), lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);


			wait_ms(1000);

			sr_ecp_msg->message("Go up");

			set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/up_to_p0.trj", lib::irp6p_m::ROBOT_NAME);
			wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

			wait_ms(1000);
		}

		sr_ecp_msg->message("Reaching building place...");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_JOINT_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/pos_build_start.trj", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		wait_ms(1000);

		int param = 100 * present_position[0] + 10 * present_position[1] + present_position[2];

		sr_ecp_msg->message("Reaching position...");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_NEWSMOOTH, param, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		wait_ms(1000);

		sr_ecp_msg->message("Force approach");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, (int) ecp_mp::generator::tff_gripper_approach::behaviour_specification, ecp_mp::generator::tff_gripper_approach::behaviour_specification_data_type(0.02, 600, 2), lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		wait_ms(4000);

		sr_ecp_msg->message("Raising up...");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/build.trj", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		sr_ecp_msg->message("Force approach");
		set_next_ecp_state(ecp_mp::generator::ECP_GEN_TFF_GRIPPER_APPROACH, BUILDING, "", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		sr_ecp_msg->message("Raising up...");

		set_next_ecp_state(ecp_mp::generator::ECP_GEN_SMOOTH_ANGLE_AXIS_FILE_FROM_MP, 5, "../../src/application/block_move/trjs/up_after_pushing.trj", lib::irp6p_m::ROBOT_NAME);
		wait_for_task_termination(false, lib::irp6p_m::ROBOT_NAME);

		wait_ms(1000);

		sr_ecp_msg->message("end of main loop");
	}

	sr_ecp_msg->message("Block move END");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
