/*
 * bclike_mp.cc
 *
 *  Created on: 06-07-2010
 *      Author: kszkudla
 */

#include "bclike_mp.h"
#include "bcl_t_switcher.h"
#include "ecp_mp_st_smooth_move.h"

namespace mrrocpp {

namespace mp {

namespace task {

const double left[] = { 0.5, -1.87, 0.100, -0.040, 4.627, -1.57, 0.0};
const double right[] = { -0.55, -1.37, 0.100, -0.040, 4.627, -1.57, 0.0};
const double start[] = { 0.0, -1.37, 0.100, -0.040, 4.627, 0.0, 0.0};

bclike_mp::bclike_mp(lib::configurator &_config) :
	task(_config) {

}

bclike_mp::~bclike_mp() {
}

void bclike_mp::main_task_algorithm(void){

	sr_ecp_msg->message("MP start");

//	while(true){
//		set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::LEFT, "", 0, 1, lib::ROBOT_IRP6OT_M.c_str());
//
//		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_IRP6OT_M.c_str());
//
//		sr_ecp_msg->message("LEFT send");
//
//		set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::RIGHT, "", 0, 1, lib::ROBOT_IRP6OT_M.c_str());
//
//		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_IRP6OT_M.c_str());
//
//		sr_ecp_msg->message("RIGHT send");

	//Set robot to start position (center)
	int i = 0;
	char* tab;
	std::vector<double> vec;

//		Rozkazy dla IRP6_OT
//		set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tab, 0, 1, lib::ROBOT_IRP6OT_M.c_str());
//		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_IRP6OT_M.c_str());

//		Rozkazy dla IRP6_P
		set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, lib::ROBOT_IRP6P_M.c_str());
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6P_M.c_str(), lib::ROBOT_IRP6P_M.c_str());


	while(1){
		switch(i){
			case 0:
				vec.clear();
				vec.assign(left, left + 7);
				tab = msg.dataToString(vec);
//				std::cout << "MP ODCZYT " << ((double*)tab)[0] << std::endl;
				sr_ecp_msg->message("RIGHT send");
				break;
			case 1:
				vec.clear();
				vec.assign(right, right + 7);
				tab = msg.dataToString(vec);
				sr_ecp_msg->message("LEFT send");
				break;
			case 2:
				vec.clear();
				vec.assign(start, start + 7);
				tab = msg.dataToString(vec);
				sr_ecp_msg->message("START send");
				break;
		}

		i++;
		i = i % 2;
//
//		std::cout << "Pozycja numer: " << i << std::endl;

		set_next_ecps_state (ecp_mp::task::ECP_ST_SMOOTH_MOVE, (int)mrrocpp::ecp::common::task::START, tab, 300, 1, lib::ROBOT_IRP6P_M.c_str());
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6P_M.c_str(), lib::ROBOT_IRP6P_M.c_str());

		std::cout << "REPLY PKG " << robot_m[lib::ROBOT_IRP6P_M]->ecp_reply_package.ecp_2_mp_string << std::endl;


		sr_ecp_msg->message("MP end loop");

	}
//	set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, "hmmm, ciekawe czy dizala", 0, 1, lib::ROBOT_IRP6OT_M.c_str());
//
//	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_IRP6OT_M.c_str());
//
//	sr_ecp_msg->message("START send");

//	}

	sr_ecp_msg->message("MP end");

}


task* return_created_mp_task (lib::configurator &_config)
{
return new bclike_mp(_config);
}


}

}

}
