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
#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"
#include "base/mp/MP_main_error.h"
#include "ecp_mp_t_swarmitfix.h"
#include "mp_t_swarmitfix.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/epos/dp_epos.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_epos.h"

#include "robot/shead/mp_r_shead.h"
#include "robot/spkm/mp_r_spkm.h"
#include "robot/smb/mp_r_smb.h"

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

	ACTIVATE_MP_ROBOT(spkm);
	ACTIVATE_MP_ROBOT(smb);
	ACTIVATE_MP_ROBOT(shead);

}

swarmitfix::swarmitfix(lib::configurator &_config) :
	task(_config)
{
}

void swarmitfix::main_task_algorithm(void)
{

	sr_ecp_msg->message("New swarmitfix series");

	// wlaczenie generatora transparentnego w obu robotach
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::spkm::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::smb::ROBOT_NAME.c_str());
	set_next_ecps_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, (int) 0, "", 0, 1, lib::shead::ROBOT_NAME.c_str());

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

	int_port_from_manager->data = 16;

	int_port_from_manager->set();
	int_port_from_manager->get();

	ss << " " << int_port_from_manager->data;

	sr_ecp_msg->message(ss.str().c_str());

	send_end_motion_to_ecps(1, lib::spkm::ROBOT_NAME.c_str());
	/*
	 sr_ecp_msg->message("2");
	 set_next_ecps_state(ecp_mp::generator::ECP_GEN_SLEEP, (int) 5, "",  0,1,
	 lib::spkm::ROBOT_NAME);
	 sr_ecp_msg->message("3");
	 run_extended_empty_gen_and_wait(
	 1, 1, lib::spkm::ROBOT_NAME, lib::spkm::ROBOT_NAME);
	 */
	sr_ecp_msg->message("4");

	char tmp_string[lib::MP_2_ECP_STRING_SIZE];

	lib::epos::epos_cubic_command epos_params;

	epos_params.da[4] = 3.7;

	memcpy(tmp_string, &epos_params, sizeof(epos_params));

	set_next_ecps_state(ecp_mp::generator::ECP_GEN_EPOS_CUBIC, (int) 5, tmp_string, sizeof(epos_params), 1, lib::spkm::ROBOT_NAME.c_str());
	sr_ecp_msg->message("5");
	run_extended_empty_gen_and_wait(1, 1, lib::spkm::ROBOT_NAME.c_str(), lib::spkm::ROBOT_NAME.c_str());

	sr_ecp_msg->message("END");

	send_end_motion_to_ecps(2, lib::smb::ROBOT_NAME.c_str(), lib::shead::ROBOT_NAME.c_str());

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
