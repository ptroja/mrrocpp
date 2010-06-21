/*
 * mp_t_visualservo_tester.cpp
 *
 *  Created on: May 28, 2010
 *      Author: mboryn
 */

#include "mp_t_visualservo_tester.h"
#include "lib/robot_consts/irp6ot_m_const.h"
#include "robot/conveyor/conveyor_const.h"

namespace mrrocpp {

namespace mp {

namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new visualservo_tester(_config);
}

visualservo_tester::visualservo_tester(lib::configurator &config) :
	task(config)
{
}

visualservo_tester::~visualservo_tester()
{
}

void visualservo_tester::main_task_algorithm(void)
{
	set_next_ecps_state(0, 0, "", 0, 1, lib::ROBOT_IRP6OT_M.c_str());
	set_next_ecps_state(0, 0, "", 0, 1, lib::ROBOT_CONVEYOR.c_str());

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2, lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_CONVEYOR.c_str(), lib::ROBOT_IRP6OT_M.c_str(), lib::ROBOT_CONVEYOR.c_str());
}

}//namespace task

}

}
