/*
 * mp_t_visualservo_tester.cpp
 *
 *  Created on: May 28, 2010
 *      Author: mboryn
 */
#include "base/mp/mp_task.h"
#include "mp_t_visualservo_tester.h"

#include "robot/conveyor/const_conveyor.h"

#include "base/lib/logger.h"

#include "../ecp_mp_g_visual_servo_tester.h"

#include <unistd.h>

#include "../defines.h"

using namespace logger;

namespace mrrocpp {

namespace mp {

namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new visualservo_tester(_config);
}

visualservo_tester::visualservo_tester(lib::configurator &config) :
	task(config), config_section_name("[visualservo_tester]")
{
	run_vs = config.value <bool> ("run_vs", config_section_name);
	run_conveyor = config.value <bool> ("run_conveyor", config_section_name);
	vs_settle_time = config.value <int> ("vs_settle_time", config_section_name);
}

visualservo_tester::~visualservo_tester()
{
}

void visualservo_tester::main_task_algorithm(void)
{
	if (run_vs) {
		sr_ecp_msg->message("Starting visual servo");
		set_next_ecps_state(mrrocpp::ecp_mp::common::generator::ECP_GEN_VISUAL_SERVO_TEST, 0, "", 0, 1, ROBOT_NAME.c_str());
		sr_ecp_msg->message("Visual servo started.");

		char txt[128];
		sprintf(txt, "Waiting for settle down (%d s)", vs_settle_time);
		sr_ecp_msg->message(txt);
		for (int i = vs_settle_time; i > 0; --i) {
			log("Waiting for VS to stabilize %-4d   \r", i);
			sleep(1);
		}
		log("\n");
	}

	if (run_conveyor) {
		sr_ecp_msg->message("Starting conveyor");

		set_next_ecps_state(mrrocpp::ecp_mp::common::generator::ECP_GEN_CONVEYOR_VS_TEST, 0, "", 0, 1, lib::ROBOT_CONVEYOR.c_str());

		sr_ecp_msg->message("Conveyor started.");
	}

	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(2, 2, ROBOT_NAME.c_str(), lib::ROBOT_CONVEYOR.c_str(), ROBOT_NAME.c_str(), lib::ROBOT_CONVEYOR.c_str());

	log("visualservo_tester::main_task_algorithm() 4\n");
}

}//namespace task

}

}
