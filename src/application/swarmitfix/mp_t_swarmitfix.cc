#include <string>

#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "base/lib/sr/srlib.h"

#include "base/mp/mp_task.h"
#include "base/mp/MP_main_error.h"
#include "mp_t_swarmitfix.h"
#include "ecp_mp_g_spkm.h"

#include "robot/shead/mp_r_shead1.h"
#include "robot/shead/mp_r_shead2.h"
#include "robot/spkm/mp_r_spkm1.h"
#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb1.h"
#include "robot/smb/mp_r_smb2.h"

#include "base/lib/swarmtypes.h"
#include "robot/spkm/dp_spkm.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix(_config);
}

swarmitfix::swarmitfix(lib::configurator &_config) :
		task(_config)
{
	// Initialize internal memory variables
	current_plan_status = ONGOING;

	// Call the robot activation so we can support only the active ones
	create_robots();

	// Initialize status of the active robots
	BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
	{
		current_workers_status[robot_node.second->robot_name] = IDLE;
	}
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void swarmitfix::create_robots()
{
	if(IS_MP_ROBOT_ACTIVE(spkm1)) IO.transmitters.spkm1.notification.Create(*this, lib::spkm1::ROBOT_NAME+lib::notifyBufferId);
	if(IS_MP_ROBOT_ACTIVE(spkm2)) IO.transmitters.spkm2.notification.Create(*this, lib::spkm2::ROBOT_NAME+lib::notifyBufferId);
	if(IS_MP_ROBOT_ACTIVE(smb1)) IO.transmitters.smb1.notification.Create(*this, lib::smb1::ROBOT_NAME+lib::notifyBufferId);
	if(IS_MP_ROBOT_ACTIVE(smb2)) IO.transmitters.smb2.notification.Create(*this, lib::smb2::ROBOT_NAME+lib::notifyBufferId);

	ACTIVATE_MP_ROBOT(spkm1);
	ACTIVATE_MP_ROBOT(spkm2);
	ACTIVATE_MP_ROBOT(smb1);
	ACTIVATE_MP_ROBOT(smb2);
//	ACTIVATE_MP_ROBOT(shead1);
//	ACTIVATE_MP_ROBOT(shead2);

	if(is_robot_activated(lib::spkm1::ROBOT_NAME)) {
		IO.transmitters.spkm1.command.Create(robot_m[lib::spkm1::ROBOT_NAME]->ecp, lib::commandBufferId);
	}
	if(is_robot_activated(lib::spkm2::ROBOT_NAME)) {
		IO.transmitters.spkm1.command.Create(robot_m[lib::spkm2::ROBOT_NAME]->ecp, lib::commandBufferId);
	}
	if(is_robot_activated(lib::smb1::ROBOT_NAME)) {
		IO.transmitters.spkm1.command.Create(robot_m[lib::smb1::ROBOT_NAME]->ecp, lib::commandBufferId);
	}
	if(is_robot_activated(lib::smb2::ROBOT_NAME)) {
		IO.transmitters.spkm1.command.Create(robot_m[lib::smb2::ROBOT_NAME]->ecp, lib::commandBufferId);
	}
}

void swarmitfix::main_task_algorithm(void)
{
	sr_ecp_msg->message("swarmitfix task started");

	do {
		if (current_plan_status == ONGOING) {
			do {
				if(ReceiveSingleMessage(false)) {
					if(current_plan_status != ONGOING)
						break;
				}

				if(robot_m[lib::spkm1::ROBOT_NAME]->reply.isFresh()) {
					robot_m[lib::spkm1::ROBOT_NAME]->reply.markAsUsed();

					current_workers_status[lib::spkm1::ROBOT_NAME] = IDLE;
				}

			} while (true);

			// ...
			continue;
		}

		if (current_plan_status == FAILURE) {
			// ...
			continue;
		}

		// Fallback to wait for change in the agent state
		ReceiveSingleMessage(true);
	} while(true);

	sr_ecp_msg->message("END");
}

} // namespace task
} // namespace mp
} // namespace mrrocpp
