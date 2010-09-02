// -------------------------------------------------------------------------
//                              task/mp_t_haptic.cc
//
// MP task for two robot haptic device
//
// -------------------------------------------------------------------------

#include <iostream>
#include <string>
#include <sstream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/srlib.h"

#include "base/mp/mp_task.h"
#include "base/mp/MP_main_error.h"
#include "ecp_mp_t_swarmitfix.h"
#include "mp_t_swarmitfix.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/epos/dp_epos.h"
#include "robot/smb/const_smb.h"
#include "robot/spkm/const_spkm.h"
#include "robot/shead/const_shead.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_epos.h"

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

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix(_config);
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void swarmitfix::create_robots()
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

swarmitfix::swarmitfix(lib::configurator &_config) :
	task(_config)
{
}

void swarmitfix::main_task_algorithm(void)
{

	sr_ecp_msg->message("New swarmitfix series");

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state(ecp_mp::common::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::spkm::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::common::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::smb::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::common::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::shead::ROBOT_NAME.c_str());

	double a = 2.88;

	std::string astring;

	astring = "11";
	std::stringstream ss(std::stringstream::in | std::stringstream::out);
	ss << a;
	astring = ss.str();
	lib::single_thread_port_manager port_manager;

	lib::single_thread_port <int> int_port("int_port_label", port_manager);
	lib::single_thread_port <int>* int_port_from_manager;

	int_port_from_manager = port_manager.get_port <int> ("int_port_label");

	int int_port_data_input = 16;
	int int_port_data_output;

	int_port_from_manager->set(int_port_data_input);
	int_port_from_manager->get(int_port_data_output);

	ss << " " << int_port_data_output;

	sr_ecp_msg->message(ss.str().c_str());

	send_end_motion_to_ecps(1, lib::spkm::ROBOT_NAME.c_str());
	/*
	 sr_ecp_msg->message("2");
	 set_next_ecps_state(ecp_mp::common::generator::ECP_GEN_SLEEP, (int) 5, "",  0,1,
	 lib::spkm::ROBOT_NAME);
	 sr_ecp_msg->message("3");
	 run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(
	 1, 1, lib::spkm::ROBOT_NAME, lib::spkm::ROBOT_NAME);
	 */
	sr_ecp_msg->message("4");

	char tmp_string[lib::MP_2_ECP_NEXT_STATE_STRING_SIZE];

	lib::epos::epos_cubic_command epos_params;

	epos_params.da[4] = 3.7;

	memcpy(tmp_string, &epos_params, sizeof(epos_params));

	set_next_ecps_state(ecp_mp::common::generator::ECP_GEN_EPOS_CUBIC, (int) 5, tmp_string, sizeof(epos_params), 1, lib::spkm::ROBOT_NAME.c_str());
	sr_ecp_msg->message("5");
	run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots(1, 1, lib::spkm::ROBOT_NAME.c_str(), lib::spkm::ROBOT_NAME.c_str());

	sr_ecp_msg->message("END");

	send_end_motion_to_ecps(2, lib::smb::ROBOT_NAME.c_str(), lib::shead::ROBOT_NAME.c_str());

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
