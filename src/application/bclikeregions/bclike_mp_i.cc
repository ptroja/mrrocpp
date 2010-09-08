/*
 * bclike_mp.cc
 *
 *  Created on: 06-07-2010
 *      Author: kszkudla
 */

#include "bclike_mp_i.h"
#include "bcl_t_switcher.h"
#include "ecp_mp_st_scan_move.h"
#include "ecp_mp_st_position_move.h"

#include "base/mp/mp_robot.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"

namespace mrrocpp {

namespace mp {

namespace task {

#ifdef IRP6_OT
	const lib::robot_name_t actual_robot = lib::irp6ot_m::ROBOT_NAME;
#endif

#ifdef IRP6_P
	const lib::robot_name_t actual_robot = lib::irp6p_m::ROBOT_NAME;
#endif


/**
 * Main Process class constructor
 * @param _config reference to configuration file parser object
 */
bclike_mp_i::bclike_mp_i(lib::configurator &_config) :
	task(_config) {

	second_task = config.value<std::string>("fradia_task", "[vsp_second_task]");
	std::cout << "DRUGI TASK: " << second_task << std::endl;

}

void bclike_mp_i::create_robots()
{
#ifdef IRP6_OT
	ACTIVATE_MP_ROBOT(irp6ot_m);
#endif

#ifdef IRP6_P
	ACTIVATE_MP_ROBOT(irp6p_m);
#endif
}

bclike_mp_i::~bclike_mp_i() {
}

/**
 * Class main method responsible for switching between subtask
 * depending on received data
 */
void bclike_mp_i::main_task_algorithm(void){

	sr_ecp_msg->message("MP start");

	char* tab;
	char* tmp;
	std::vector<double> vec;

//	Set robot to start position (center)
	vec.clear();
	vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);
	tab = msg.robotPositionToString(vec);

	set_next_ecps_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tab, 300, 1, actual_robot.c_str());
	sr_ecp_msg->message("MOVE left");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());

	//Setup end position
	vec.clear();
	vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
	tab = msg.robotPositionToString(vec);

	//Start moving
	set_next_ecps_state (ecp_mp::task::ECP_ST_SCAN_MOVE, 0, tab, 300, 1, actual_robot.c_str());
	sr_ecp_msg->message("MOVE right");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());


	std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> >::iterator it;

	while(strcmp(robot_m[actual_robot]->ecp_reply_package.ecp_2_mp_string, "KONIEC") != 0){

		pos = msg.stringToECPOrder(robot_m[actual_robot]->ecp_reply_package.ecp_2_mp_string, regions);

		for(it = regions.begin(); it!= regions.end(); ++it){
			if(!(*it).second){
				std::cout << "WYKONANIE ODCZYTU" << std::endl;
				vec[0] = (*it).first.x;
				vec[1] = (*it).first.y;
				//TODO: Przelaczyc Task we FraDIA
				//TODO: Wywolac subtask Marcina
				(*it).second = true;

				//Go to code
				tmp = msg.robotPositionToString(vec);
				set_next_ecps_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tmp, 300, 1, actual_robot.c_str());
				run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());

				//Get back to previous position
				tmp = msg.robotPositionToString(pos);
				set_next_ecps_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tmp, 300, 1, actual_robot.c_str());
				run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());
			}
		}

		set_next_ecps_state (ecp_mp::task::ECP_ST_SCAN_MOVE, 0, tab, 300, 1, actual_robot.c_str());
		sr_ecp_msg->message("MOVE right");
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());
	}

	sr_ecp_msg->message("KONIEC");

}


task* return_created_mp_task (lib::configurator &_config)
{
return new bclike_mp_i(_config);
}


}

}

}
