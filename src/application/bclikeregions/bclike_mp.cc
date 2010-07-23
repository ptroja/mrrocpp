/*
 * bclike_mp.cc
 *
 *  Created on: 06-07-2010
 *      Author: kszkudla
 */

#include "bclike_mp.h"
#include "bcl_t_switcher.h"

namespace mrrocpp {

namespace mp {

namespace task {

bclike_mp::bclike_mp(lib::configurator &_config) :
	task(_config) {
	// TODO Auto-generated constructor stub

}

bclike_mp::~bclike_mp() {
	// TODO Auto-generated destructor stub
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
	set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, "hmmm, ciekawe czy dizala", 0, 1, lib::ROBOT_IRP6OT_M.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_IRP6OT_M.c_str());

	sr_ecp_msg->message("START send");

//	set_next_ecps_state (ecp_mp::task::BCL_MOTION_DIR_STR, (int)mrrocpp::ecp::common::task::START, "hmmm, ciekawe czy dizala", 0, 1, lib::ROBOT_IRP6OT_M.c_str());
//
//	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_IRP6OT_M.c_str());
//
//	sr_ecp_msg->message("START send");

//	while(1){
//		char *qq = robot_m[lib::ROBOT_SPEECHRECOGNITION]->ecp_reply_package.ecp_2_mp_string;
//
//	}

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
