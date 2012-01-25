/**
 * \file bclike_mp_i.cc
 * \brief MP process class methods definition: INTERRUPTED_MOVE
 * \date 02.09.2010
 * \author Kacper Szkudlarek
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



bclike_mp_i::bclike_mp_i(lib::configurator &_config) :
	task(_config) {

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


void bclike_mp_i::main_task_algorithm(void){

	sr_ecp_msg->message("MP start");

	char* tab;
	char* tmp;
	std::vector<double> vec;

//	Set robot to start position (center)
	vec.clear();
	vec.assign(ecp::common::task::left, ecp::common::task::left + VEC_SIZE);
	tab = msg.robotPositionToString(vec);

	set_next_ecp_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tab, lib::ECP_2_MP_STRING_SIZE, actual_robot);
	sr_ecp_msg->message("MOVE left");
	wait_for_task_termination(false, 1,  actual_robot.c_str());

	//Setup end position
	vec.clear();
	vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
	tab = msg.robotPositionToString(vec);

	//Start moving
	set_next_ecp_state (ecp_mp::task::ECP_ST_SCAN_MOVE, 0, tab, lib::ECP_2_MP_STRING_SIZE, actual_robot);
	sr_ecp_msg->message("MOVE right");
	wait_for_task_termination(false, 1,  actual_robot.c_str());


	std::vector<std::pair<ecp::common::task::mrrocpp_regions, bool> >::iterator it;

	while(strcmp(robot_m[actual_robot]->ecp_reply_package.recognized_command, "KONIEC") != 0){

		pos = msg.stringToECPOrder(robot_m[actual_robot]->ecp_reply_package.recognized_command, regions);

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
				set_next_ecp_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tmp, lib::ECP_2_MP_STRING_SIZE, actual_robot);
				wait_for_task_termination(false, 1,  actual_robot.c_str());

				//Get back to previous position
				tmp = msg.robotPositionToString(pos);
				set_next_ecp_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tmp, lib::ECP_2_MP_STRING_SIZE, actual_robot);
				wait_for_task_termination(false, 1,  actual_robot.c_str());
			}
		}

		set_next_ecp_state (ecp_mp::task::ECP_ST_SCAN_MOVE, 0, tab, lib::ECP_2_MP_STRING_SIZE, actual_robot);
		sr_ecp_msg->message("MOVE right");
		wait_for_task_termination(false, 1, actual_robot.c_str());
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
