// Start of user code user defined headers
#include <boost/foreach.hpp>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "ecp_mp_g_spkm.h"

#include "robot/shead/mp_r_shead1.h"
#include "robot/shead/mp_r_shead2.h"
#include "robot/spkm/mp_r_spkm1.h"
#include "robot/spkm/mp_r_spkm2.h"
#include "robot/smb/mp_r_smb1.h"
#include "robot/smb/mp_r_smb2.h"
// End of user code

#include "base/lib/sr/srlib.h"
#include "base/mp/mp_task.h"

#include "mp_t_swarmitfix.h"

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
	// Create optional Input buffers
	if(IS_MP_ROBOT_ACTIVE(spkm2)) {
		IO.transmitters.spkm2.inputs.notification.Create(*this, lib::spkm2::ROBOT_NAME+"notification");
	}
	if(IS_MP_ROBOT_ACTIVE(smb2)) {
		IO.transmitters.smb2.inputs.notification.Create(*this, lib::smb2::ROBOT_NAME+"notification");
	}
	if(IS_MP_ROBOT_ACTIVE(spkm1)) {
		IO.transmitters.spkm1.inputs.notification.Create(*this, lib::spkm1::ROBOT_NAME+"notification");
	}
	if(IS_MP_ROBOT_ACTIVE(smb1)) {
		IO.transmitters.smb1.inputs.notification.Create(*this, lib::smb1::ROBOT_NAME+"notification");
	}

	// Call the robot activation so we can support only the active ones
	create_robots();

	// Create optional Output buffers
	if(is_robot_activated(lib::spkm2::ROBOT_NAME)) {
		IO.transmitters.spkm2.outputs.command.Create(robot_m[lib::spkm2::ROBOT_NAME]->ecp, "command");
	}
	if(is_robot_activated(lib::spkm1::ROBOT_NAME)) {
		IO.transmitters.spkm1.outputs.command.Create(robot_m[lib::spkm1::ROBOT_NAME]->ecp, "command");
	}
	
	// Start of user code Initialize internal memory variables
// End of user code
}

// powolanie robotow w zaleznosci od zawartosci pliku konfiguracyjnego
void swarmitfix::create_robots()
{
	ACTIVATE_MP_ROBOT(smb2);
	ACTIVATE_MP_ROBOT(smb1);
	ACTIVATE_MP_ROBOT(spkm1);
	ACTIVATE_MP_ROBOT(spkm2);
}

void swarmitfix::main_task_algorithm(void)
{
	sr_ecp_msg->message("task started");

	do {
		if (b1_initial_condition() == true) {
			do {
				if(b1_terminal_condition() == true)
					break;
					
				if(ReceiveSingleMessage(false)) {
					if(b1_terminal_condition() == true)					
						break;
				}

				// Check if right set of data is available
				if(IO.sensors.planner.inputs.trigger->isFresh())
				{
					// Mark data in buffers as used 
					IO.sensors.planner.inputs.trigger->markAsUsed();

					// Call the transition function routine
					b1_plan_progress();
					
					continue;
				}
				
				// Check if right set of data is available
				if(IO.transmitters.spkm1.inputs.notification->isFresh())
				{
					// Mark data in buffers as used 
					IO.transmitters.spkm1.inputs.notification->markAsUsed();

					// Call the transition function routine
					b1_handle_spkm1_notification();
					
					continue;
				}
				
				// Check if right set of data is available
				if(IO.transmitters.spkm2.inputs.notification->isFresh())
				{
					// Mark data in buffers as used 
					IO.transmitters.spkm2.inputs.notification->markAsUsed();

					// Call the transition function routine
					b1_handle_spkm2_notification();
					
					continue;
				}
				
				// Check if right set of data is available
				if(IO.transmitters.smb1.inputs.notification->isFresh())
				{
					// Mark data in buffers as used 
					IO.transmitters.smb1.inputs.notification->markAsUsed();

					// Call the transition function routine
					b1_handle_smb1_notification();
					
					continue;
				}
				
				// Check if right set of data is available
				if(IO.transmitters.smb2.inputs.notification->isFresh())
				{
					// Mark data in buffers as used 
					IO.transmitters.smb2.inputs.notification->markAsUsed();

					// Call the transition function routine
					b1_handle_smb2_notification();
					
					continue;
				}
				
				// Blocking for a new message has the lowest priority 
				ReceiveSingleMessage(true);
			} while (true);

			// ...
			continue;
		}
		if (b2_initial_condition() == true) {
			do {
				if(b2_terminal_condition() == true)
					break;
					
				if(ReceiveSingleMessage(false)) {
					if(b2_terminal_condition() == true)					
						break;
				}

				// There is no data dependency for this transition function				
				{

					// Call the transition function routine
					b2_stop_all();
					
					continue;
				}
				
				// Blocking for a new message has the lowest priority 
				ReceiveSingleMessage(true);
			} while (true);

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
