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

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"

#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"

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
	run_vs = config.value <bool>("run_vs", config_section_name);
	run_conveyor = config.value <bool>("run_conveyor", config_section_name);
	vs_settle_time = config.value <int>("vs_settle_time", config_section_name);
	robot_name = config.value <std::string>("robot_name", config_section_name);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void visualservo_tester::create_robots()
{
	ACTIVATE_MP_ROBOT(conveyor);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_m);
}

visualservo_tester::~visualservo_tester()
{
}

void visualservo_tester::main_task_algorithm(void)
{
	if (run_vs) {
		sr_ecp_msg->message("Starting visual servo");
		set_next_ecp_state(mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST, 0, "", robot_name);
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

		set_next_ecp_state(mrrocpp::ecp_mp::generator::ECP_GEN_CONVEYOR_VS_TEST, 0, "", lib::conveyor::ROBOT_NAME);

		sr_ecp_msg->message("Conveyor started.");
	}

	wait_for_task_termination(false, robot_name, lib::conveyor::ROBOT_NAME);

	log("visualservo_tester::main_task_algorithm() 4\n");
}

} //namespace task

}

}
