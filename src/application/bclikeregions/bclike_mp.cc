/*
 * bclike_mp.cc
 *
 *  Created on: 06-07-2010
 *      Author: kszkudla
 */

#include "bclike_mp.h"
#include "bcl_t_switcher.h"
#include "ecp_mp_st_smooth_move.h"

#include "base/mp/mp_robot.h"

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
bclike_mp::bclike_mp(lib::configurator &_config) :
	task(_config) {

	second_task = config.value<std::string>("fradia_task", "[vsp_second_task]");
	std::cout << "DRUGI TASK: " << second_task << std::endl;

	movement_type = config.value<int>("robot_movement", "[bcl_taks]");
	std::cout << "MOVEMENT TYPE: " << movement_type << std::endl;

	//TODO: Wczytywanie z konfiga informacji o drugim zadaniu

}

bclike_mp::~bclike_mp() {
}

/**
 * Class main method responsible for switching between subtask
 * depending on received data
 */
void bclike_mp::main_task_algorithm(void){

	sr_ecp_msg->message("MP start");

	char* tab;
	char* tmp;
	std::vector<double> vec;

//	Set robot to start position (center)
	vec.clear();
	vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);
	tab = msg.robotPositionToString(vec);

	set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
	sr_ecp_msg->message("MOVE left");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());


	if(movement_type == 3){
		int i = 0;

		//Move robot between three control points continously
		while(1){
			switch(i){
				case 0:
					vec.clear();
					vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);
					tab = msg.robotPositionToString(vec);
					sr_ecp_msg->message("RIGHT send");
					break;
				case 1:
					vec.clear();
					vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
					tab = msg.robotPositionToString(vec);
					sr_ecp_msg->message("LEFT send");
					break;
				case 2:
					vec.clear();
					vec.assign(ecp::common::task::start, ecp::common::task::start + VEC_SIZE);
					tab = msg.robotPositionToString(vec);
					sr_ecp_msg->message("START send");
					break;
			}

			i++;
			i = i % 3;

			set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
			run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());

			sr_ecp_msg->message("MP end loop");

		}

		sr_ecp_msg->message("MP end");

		return;
	}//if test mode

	//Setup end position
	vec.clear();
	vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
	tab = msg.robotPositionToString(vec);

	//Start moving
	set_next_ecps_state (ecp_mp::task::ECP_ST_SMOOTH_MOVE, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
	sr_ecp_msg->message("MOVE right");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());


	if(movement_type == 1){
		while(strcmp(robot_m[actual_robot]->ecp_reply_package.ecp_2_mp_string, "KONIEC") != 0){

			msg.stringToECPOrder(robot_m[actual_robot]->ecp_reply_package.ecp_2_mp_string, regions);

	//		std::cout << "ODCZYT: " << regions.size() << std::endl;

			set_next_ecps_state (ecp_mp::task::ECP_ST_SMOOTH_MOVE, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
			sr_ecp_msg->message("MOVE right");
			run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());
		}
		sr_ecp_msg->message("KONIEC RUCHU");
		std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> >::iterator it;

		std::cout << "WYWOLANIA KODOW: " << regions.size() << std::endl;
		for(it = regions.begin(); it != regions.end(); ++it){
			if(!(*it).second){
				//Create vector with code position
				vec[0] = (*it).first.x;
				vec[1] = (*it).first.y;

				(*it).second = true;

		//		std::cout << "x = " << vec[0] << " y = " << vec[1] << " z = " << vec[2] << std::endl;

				//Move to code position
				tmp = msg.robotPositionToString(vec);
				set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tmp, 300, 1, actual_robot.c_str());
				run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());

				//TODO: przelaczyc zadanie FrDIA
				//TODO: wywolac subtaks Marcina
			}
		}
	} else if(movement_type == 2){
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
					set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tmp, 300, 1, actual_robot.c_str());
					run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());

					//Get back to previous position
					tmp = msg.robotPositionToString(pos);
					set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tmp, 300, 1, actual_robot.c_str());
					run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());
				}
			}

			set_next_ecps_state (ecp_mp::task::ECP_ST_SMOOTH_MOVE, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, actual_robot.c_str());
			sr_ecp_msg->message("MOVE right");
			run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());
		}
	} //if movement_type

	sr_ecp_msg->message("KONIEC");

}


task* return_created_mp_task (lib::configurator &_config)
{
return new bclike_mp(_config);
}


}

}

}
