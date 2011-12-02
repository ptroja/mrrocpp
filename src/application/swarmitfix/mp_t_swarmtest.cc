#include "mp_t_swarmitfix.h"

#include "planner.h"
#include "base/lib/mrmath/homog_matrix.h"
#include "base/mp/mp_exceptions.h"

namespace mrrocpp {
namespace mp {
namespace task {

void swarmitfix::main_test_algorithm(void)
{
	sr_ecp_msg->message("swarm test started");

	// Create planner object
	// planner pp(config.value<std::string>("planpath"));

	sr_ecp_msg->message("plan OK");

	const Plan * p = pp.getPlan();

	for(Plan::PkmType::ItemConstIterator it = p->pkm().item().begin();
			it != p->pkm().item().end();
			++it) {

		const Plan::PkmType::ItemType & pkmCmd = *it;

		// Test only 1st agent
		if(pkmCmd.agent() != 1)
			continue;

		// Goal pose
		lib::Homog_matrix hm;

		if(pkmCmd.pkmToWrist().present()) {
			hm = lib::Homog_matrix(pkmCmd.pkmToWrist().get());
		} else if (pkmCmd.Xyz_Euler_Zyz().present()) {
			hm = lib::Xyz_Euler_Zyz_vector(
					pkmCmd.Xyz_Euler_Zyz()->x(),
					pkmCmd.Xyz_Euler_Zyz()->y(),
					pkmCmd.Xyz_Euler_Zyz()->z(),
					pkmCmd.Xyz_Euler_Zyz()->ox(),
					pkmCmd.Xyz_Euler_Zyz()->oy(),
					pkmCmd.Xyz_Euler_Zyz()->oz()
					);
		}

		// Setup variant for the PKM
		lib::spkm::next_state_t cmd(lib::spkm::POSE_LIST);

		cmd.segments.push_back(hm);

		std::cerr << "HM is\n" << hm << std::endl;

		// Send command with the output buffer
		IO.transmitters.spkm2.outputs.command->Send(cmd);

		while(!IO.transmitters.spkm2.inputs.notification->isFresh()) {
			std::cout << "MP blocking for SPKM2 notification" << std::endl;
			ReceiveSingleMessage(true);
		}

		IO.transmitters.spkm2.inputs.notification->markAsUsed();

		if(IO.transmitters.spkm2.inputs.notification->Get() == lib::NACK) {
			BOOST_THROW_EXCEPTION(exception::nfe());
		}
	}

	return;



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
