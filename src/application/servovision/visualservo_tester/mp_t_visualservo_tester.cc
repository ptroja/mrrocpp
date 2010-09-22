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

#include "robot/conveyor/mp_r_conveyor.h"
#include "robot/irp6ot_m/mp_r_irp6ot_m.h"
#include "robot/irp6p_m/mp_r_irp6p_m.h"
#include "robot/irp6m/mp_r_irp6m.h"
#include "robot/speaker/mp_r_speaker.h"
#include "robot/polycrank/mp_r_polycrank.h"
#include "robot/bird_hand/mp_r_bird_hand.h"
#include "robot/irp6ot_tfg/mp_r_irp6ot_tfg.h"
#include "robot/irp6p_tfg/mp_r_irp6p_tfg.h"
#include "robot/shead/mp_r_shead.h"
#include "robot/spkm/mp_r_spkm.h"
#include "robot/smb/mp_r_smb.h"
#include "robot/sarkofag/mp_r_sarkofag.h"
#include "robot/festival/const_festival.h"
#include "robot/player/const_player.h"

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

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void visualservo_tester::create_robots()
{

	ACTIVATE_MP_ROBOT(conveyor);
	ACTIVATE_MP_ROBOT(speaker);
	ACTIVATE_MP_ROBOT(irp6m);
	ACTIVATE_MP_ROBOT(polycrank);
	ACTIVATE_MP_ROBOT(bird_hand);
	ACTIVATE_MP_ROBOT(spkm);
	ACTIVATE_MP_ROBOT(smb);
	ACTIVATE_MP_ROBOT(shead);
	ACTIVATE_MP_ROBOT(irp6ot_tfg);
	ACTIVATE_MP_ROBOT(irp6ot_m);
	ACTIVATE_MP_ROBOT(irp6p_tfg);
	ACTIVATE_MP_ROBOT(irp6p_m);
	ACTIVATE_MP_ROBOT(sarkofag);

	ACTIVATE_MP_DEFAULT_ROBOT(electron);
	ACTIVATE_MP_DEFAULT_ROBOT(speechrecognition);
	ACTIVATE_MP_DEFAULT_ROBOT(festival);

}

visualservo_tester::~visualservo_tester()
{
}

void visualservo_tester::main_task_algorithm(void)
{
	if (run_vs) {
		sr_ecp_msg->message("Starting visual servo");
		set_next_ecps_state(mrrocpp::ecp_mp::generator::ECP_GEN_VISUAL_SERVO_TEST, 0, "", 0, 1, ROBOT_NAME_MB.c_str());
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

		set_next_ecps_state(mrrocpp::ecp_mp::generator::ECP_GEN_CONVEYOR_VS_TEST, 0, "", 0, 1, lib::conveyor::ROBOT_NAME.c_str());

		sr_ecp_msg->message("Conveyor started.");
	}

	run_extended_empty_gen_and_wait(2, 2, ROBOT_NAME_MB.c_str(), lib::conveyor::ROBOT_NAME.c_str(), ROBOT_NAME_MB.c_str(), lib::conveyor::ROBOT_NAME.c_str());

	log("visualservo_tester::main_task_algorithm() 4\n");
}

}//namespace task

}

}
