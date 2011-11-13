#include <iostream>
#include <string>
#include <sstream>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"
#include "base/mp/MP_main_error.h"
#include "mp_t_swarmitfix.h"
#include "base/lib/single_thread_port.h"
#include "base/lib/mrmath/mrmath.h"
#include "robot/maxon/dp_epos.h"
#include "generator/ecp/ecp_mp_g_transparent.h"
#include "ecp_mp_g_spkm.h"

#include "robot/shead/mp_r_shead1.h"
#include "robot/shead/mp_r_shead2.h"
#include "robot/spkm/mp_r_spkm1.h"
#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb1.h"
#include "robot/smb/mp_r_smb2.h"

#include "plan.hxx"

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
	ACTIVATE_MP_ROBOT(spkm1);
	ACTIVATE_MP_ROBOT(spkm2);
	ACTIVATE_MP_ROBOT(smb1);
	ACTIVATE_MP_ROBOT(smb2);
	ACTIVATE_MP_ROBOT(shead1);
	ACTIVATE_MP_ROBOT(shead2);
}

swarmitfix::swarmitfix(lib::configurator &_config) :
		task(_config)
{

}

void swarmitfix::main_task_algorithm(void)
{
	sr_ecp_msg->message("New swarmitfix series");

	// Initiate command execution on the ECP side
	set_next_ecp_state(ecp_mp::spkm::generator::ECP_GEN_POSE_LIST, 0, "", 0, lib::spkm1::ROBOT_NAME);

	robot_m[lib::spkm1::ROBOT_NAME]->mp_command.command = lib::NEXT_STATE;
	strcpy(robot_m[lib::spkm1::ROBOT_NAME]->mp_command.ecp_next_state.next_state, ecp_mp::spkm::generator::ECP_GEN_POSE_LIST.c_str());
	robot_m[lib::spkm1::ROBOT_NAME]->mp_command.ecp_next_state.spkm_segment_sequence.clear();

	return;

	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, 0, "", 0, lib::smb1::ROBOT_NAME);
	set_next_ecp_state(ecp_mp::generator::ECP_GEN_TRANSPARENT, 0, "", 0, lib::shead1::ROBOT_NAME);



	send_end_motion_to_ecps(1, lib::spkm1::ROBOT_NAME.c_str());

	sr_ecp_msg->message("4");


	wait_for_task_termination(false, 1, lib::spkm1::ROBOT_NAME.c_str());

	sr_ecp_msg->message("END");

	send_end_motion_to_ecps(2, lib::smb1::ROBOT_NAME.c_str(), lib::shead1::ROBOT_NAME.c_str());
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
