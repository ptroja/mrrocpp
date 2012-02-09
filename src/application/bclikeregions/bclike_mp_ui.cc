/**
 * \file bclike_mp_ui.cc
 * \brief MP process class methods definition: UNINTERRUPTED_MOE
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#include "bclike_mp_ui.h"
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



bclike_mp_ui::bclike_mp_ui(lib::configurator &_config) :
	task(_config) {

}

void bclike_mp_ui::create_robots()
{
#ifdef IRP6_OT
	ACTIVATE_MP_ROBOT(irp6ot_m);
#endif

#ifdef IRP6_P
	ACTIVATE_MP_ROBOT(irp6p_m);
#endif
}

bclike_mp_ui::~bclike_mp_ui() {
}


void bclike_mp_ui::main_task_algorithm(void){

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
	wait_for_task_termination(false, actual_robot);

	//Setup end position
	vec.clear();
	vec.assign(ecp::common::task::right, ecp::common::task::right + VEC_SIZE);
	tab = msg.robotPositionToString(vec);

	//Start moving
	set_next_ecp_state (ecp_mp::task::ECP_ST_SCAN_MOVE, 0, tab, lib::ECP_2_MP_STRING_SIZE, 1, actual_robot.c_str());
	sr_ecp_msg->message("MOVE right");
	wait_for_task_termination(false, actual_robot);


	while(strcmp(robot_m[actual_robot]->ecp_reply_package.recognized_command, "KONIEC") != 0){

		msg.stringToECPOrder(robot_m[actual_robot]->ecp_reply_package.recognized_command, regions);

//		std::cout << "ODCZYT: " << regions.size() << std::endl;

		set_next_ecp_state (ecp_mp::task::ECP_ST_SCAN_MOVE, 0, tab, lib::ECP_2_MP_STRING_SIZE, 1, actual_robot.c_str());
		sr_ecp_msg->message("MOVE right");
		wait_for_task_termination(false, actual_robot);
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
			set_next_ecp_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tmp, lib::ECP_2_MP_STRING_SIZE, 1, actual_robot.c_str());
			wait_for_task_termination(false, actual_robot);

			//TODO: przelaczyc zadanie FrDIA
			//TODO: wywolac subtaks Marcina
		}
	}
	sr_ecp_msg->message("KONIEC");

}


task* return_created_mp_task (lib::configurator &_config)
{
return new bclike_mp_ui(_config);
}


}

}

}
