// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/srlib.h"

#include "base/mp/mp_task.h"
#include "base/mp/MP_main_error.h"
#include "ecp_mp_t_bird_hand_test.h"
#include "mp_t_bird_hand_test.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/bird_hand/dp_bird_hand.h"
#include "robot/bird_hand/const_bird_hand.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_bird_hand_test.h"

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

#include <iostream>
#include <string>
#include <sstream>

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new bird_hand_test(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void bird_hand_test::create_robots()
{
	/*
	 * this is necessary to first create robot and then assign it to robot_m
	 * reason: mp_robot() constructor uses this map (by calling
	 * mp_task::mp_wait_for_name_open() so needs the map to be in
	 * a consistent state
	 */

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

bird_hand_test::bird_hand_test(lib::configurator &_config) :
	task(_config)
{
}

void bird_hand_test::main_task_algorithm(void)
{

	sr_ecp_msg->message("New bird_hand_test series");

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state(ecp_mp::common::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::bird_hand::ROBOT_NAME.c_str());

	send_end_motion_to_ecps(1, lib::bird_hand::ROBOT_NAME.c_str());

	sr_ecp_msg->message("4");

	set_next_ecps_state(ecp_mp::bird_hand::generator::ECP_GEN_BIRD_HAND_TEST, (int) 5, "", 0, 1, lib::bird_hand::ROBOT_NAME.c_str());
	sr_ecp_msg->message("5");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::bird_hand::ROBOT_NAME.c_str(), lib::bird_hand::ROBOT_NAME.c_str());

	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
