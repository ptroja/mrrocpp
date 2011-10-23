#include <iostream>
#include <string>
#include <sstream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"
#include "base/mp/MP_main_error.h"
#include "mp_t_swarm_demo_single_agent.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/maxon/dp_epos.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_spkm.h"
#include "ecp_mp_g_smb.h"

#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb2.h"

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
	ACTIVATE_MP_ROBOT(spkm2);
	ACTIVATE_MP_ROBOT(smb2);
}

swarmitfix::swarmitfix(lib::configurator &_config) :
		task(_config)
{

}

void swarmitfix::main_task_algorithm(void)
{

	// communication structures to send to ECP
	lib::smb::festo_command_td mp_ecp_festo_command;

	char mp_ecp_string[lib::MP_2_ECP_STRING_SIZE];

	sr_ecp_msg->message("New experimental series");

// znajdujemy sie w pozycji bazy jezdnej A
// smb - najpierw idziemy robotem w gore (wszystkie nogi w dol)
	sr_ecp_msg->message("1");

	for (int i = 0; i < lib::smb::LEG_CLAMP_NUMBER; i++) {
		mp_ecp_festo_command.leg[i] = lib::smb::DOWN;
		mp_ecp_festo_command.undetachable[i] = false;
	}

	memcpy(mp_ecp_string, &mp_ecp_festo_command, sizeof(mp_ecp_festo_command));

	set_next_ecp_state(ecp_mp::smb::generator::ECP_LEGS_COMMAND, 0, mp_ecp_string, sizeof(mp_ecp_string), lib::smb2::ROBOT_NAME);
	wait_for_task_termination(false, 1, lib::smb2::ROBOT_NAME.c_str());

// spkm - przemieszczamy manipulator do pozycji podparcia dykty
	sr_ecp_msg->message("2");

// stoimy przez dwie sekundy symulujac podparcie dykty
	sr_ecp_msg->message("3");

// spkm - przemieszczamy manipulator do pozycji marszowej (opuszczamy koncowke w dol)
	sr_ecp_msg->message("4");

// smb - unosimy dwie nogi
	sr_ecp_msg->message("5");

// smb - obracamy sie wokol opuszczonej nogi do pozycji bazy jezdnej B
	sr_ecp_msg->message("6");

// smb - opusczamy dwie nogi, ktore byly w gorze
	sr_ecp_msg->message("7");

// spkm - przemieszczamy manipulator do pozycji podparcia dykty
	sr_ecp_msg->message("8");

// stoimy przez dwie sekundy symulujac podparcie dykty
	sr_ecp_msg->message("9");

// spkm - przemieszczamy manipulator do pozycji marszowej (opuszczamy koncowke w dol)
	sr_ecp_msg->message("10");

// smb - unosimy dwie nogi
	sr_ecp_msg->message("11");

// smb - obracamy sie wokol opuszczonej nogi do pozycji bazy jezdnej A
	sr_ecp_msg->message("12");

// smb - opusczamy dwie nogi, ktore byly w gorze
	sr_ecp_msg->message("13");

// smb - podnosimy wszystkie nogi
	sr_ecp_msg->message("14");

// KONIEC
	sr_ecp_msg->message("END");

}

} // namespace task
} // namespace mp
} // namespace mrrocpp
