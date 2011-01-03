/**
 * \file bclike_mp_test.cc
 * \brief MP process class methods definition: TEST_MODE
 * \date 02.09.2010
 * \author Kacper Szkudlarek
 */

#include "bclike_mp_test.h"
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



bclike_mp_test::bclike_mp_test(lib::configurator &_config) :
	task(_config) {

}

void bclike_mp_test::create_robots()
{
#ifdef IRP6_OT
	ACTIVATE_MP_ROBOT(irp6ot_m);
#endif

#ifdef IRP6_P
	ACTIVATE_MP_ROBOT(irp6p_m);
#endif
}

bclike_mp_test::~bclike_mp_test() {
}


void bclike_mp_test::main_task_algorithm(void){

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

		set_next_ecps_state (ecp_mp::task::ECP_ST_POSITION_MOVE, 0, tab, 300, 1, actual_robot.c_str());
		run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, actual_robot.c_str(), actual_robot.c_str());

		sr_ecp_msg->message("MP end loop");

	}

	sr_ecp_msg->message("MP end");

	return;

}


task* return_created_mp_task (lib::configurator &_config)
{
return new bclike_mp_test(_config);
}


}

}

}
