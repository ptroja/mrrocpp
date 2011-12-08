#include "mp_t_swarmitfix.h"

#include "planner.h"
#include "base/lib/mrmath/homog_matrix.h"
#include "base/mp/mp_exceptions.h"

namespace mrrocpp {
namespace mp {
namespace task {

void fastForward(Plan::PkmType::ItemConstIterator & it, Plan::PkmType::ItemType::AgentType id, const Plan & p)
{
	while(it != p.pkm().item().end() && it->agent() != id) ++it;
}

void fastForward(Plan::MbaseType::ItemConstIterator & it, Plan::MbaseType::ItemType::AgentType id, const Plan & p)
{
	while(it != p.mbase().item().end() && it->agent() != id) ++it;
}

void fastForward(Plan::HeadType::ItemConstIterator & it, Plan::HeadType::ItemType::AgentType id, const Plan & p)
{
	while(it != p.head().item().end() && it->agent() != id) ++it;
}

void executeCommandItem(const Plan::PkmType::ItemType & pkmCmd, OutputBuffer<lib::spkm::next_state_t> * outBuf)
{
	// Goal pose
	lib::Homog_matrix hm;

	if(pkmCmd.pkmToWrist().present()) {
		hm = lib::Homog_matrix(pkmCmd.pkmToWrist().get());
	} else if (pkmCmd.Xyz_Euler_Zyz().present()) {

		std::cout << "Xyz_Euler_Zyz: " <<
				pkmCmd.Xyz_Euler_Zyz()->x() << " " <<
				pkmCmd.Xyz_Euler_Zyz()->y() << " " <<
				pkmCmd.Xyz_Euler_Zyz()->z() << " " <<
				pkmCmd.Xyz_Euler_Zyz()->alpha() << " " <<
				pkmCmd.Xyz_Euler_Zyz()->beta() << " " <<
				pkmCmd.Xyz_Euler_Zyz()->gamma() << std::endl;

		hm = lib::Xyz_Euler_Zyz_vector(
				pkmCmd.Xyz_Euler_Zyz()->x(),
				pkmCmd.Xyz_Euler_Zyz()->y(),
				pkmCmd.Xyz_Euler_Zyz()->z(),
				pkmCmd.Xyz_Euler_Zyz()->alpha(),
				pkmCmd.Xyz_Euler_Zyz()->beta(),
				pkmCmd.Xyz_Euler_Zyz()->gamma()
				);
	}

	// Setup variant for the PKM
	lib::spkm::next_state_t cmd(lib::spkm::POSE_LIST);

	cmd.segments.push_back(hm);
//		cmd.segments.push_back(hm);

	std::cerr << "MP # of segments = " << cmd.segments.size() << std::endl;
	for(lib::spkm::next_state_t::segment_sequence_t::const_iterator it = cmd.segments.begin();
			it != cmd.segments.end();
			++it) {
		std::cerr << "pose\n" << it->goal_pose << std::endl;
		std::cerr << "motion type " << it->motion_type << std::endl;
		std::cerr << "duration " << it->duration << std::endl;
		std::cerr << "guarded_motion " << it->guarded_motion << std::endl;
	}

	// Send command if the output buffer is active
	if(outBuf) outBuf->Send(cmd);
}

void swarmitfix::main_test_algorithm(void)
{
	sr_ecp_msg->message("swarm test started");

	// Create planner object
	// planner pp(config.value<std::string>("planpath"));

	sr_ecp_msg->message("plan OK");

	const Plan * p = pp.getPlan();

	// Plan iterators
	Plan::PkmType::ItemConstIterator spkm1_it = p->pkm().item().begin();
	Plan::PkmType::ItemConstIterator spkm2_it = p->pkm().item().begin();
	Plan::MbaseType::ItemConstIterator smb1_it = p->mbase().item().begin();
	Plan::MbaseType::ItemConstIterator smb2_it = p->mbase().item().begin();
	Plan::HeadType::ItemConstIterator shead1_it = p->head().item().begin();
	Plan::HeadType::ItemConstIterator shead2_it = p->head().item().begin();

	// Setup iterators
	fastForward(spkm1_it, 1, *p);
	fastForward(spkm2_it, 2, *p);
	fastForward(smb1_it, 1, *p);
	fastForward(smb2_it, 2, *p);
	fastForward(shead1_it, 1, *p);
	fastForward(shead2_it, 2, *p);

	// Time index counter
	int ind = 0;

	// Execute startup commands
	if(spkm1_it != p->pkm().item().end() && spkm1_it->ind() < 0) {

	}

	do {
		// Execute command for spkm1
		if(spkm1_it != p->pkm().item().end() && spkm1_it->ind() == ind) {

			executeCommandItem(*spkm1_it++, IO.transmitters.spkm1.outputs.command.get());

			// Fast-forward upto next command
			fastForward(spkm1_it, 1, *p);
		}

		// Execute command for spkm2
		if(spkm2_it != p->pkm().item().end() && spkm2_it->ind() == ind) {

			executeCommandItem(*spkm1_it++, IO.transmitters.spkm1.outputs.command.get());

			// Fast-forward upto next command
			fastForward(spkm1_it, 2, *p);
		}

		// If all iterators are at the end
		if(
				spkm1_it == p->pkm().item().end() &&
				spkm2_it == p->pkm().item().end() &&
				smb1_it == p->mbase().item().end() &&
				smb2_it == p->mbase().item().end() &&
				shead1_it == p->head().item().end() &&
				shead2_it == p->head().item().end()
				)
		{
			// Then finish
			break;
		}

	} while(++ind);
#if 0
	for(Plan::PkmType::ItemConstIterator it = p->pkm().item().begin();
			it != p->pkm().item().end();
			++it) {


		// Test only 1st agent
		if(pkmCmd.agent() != 1)
			continue;



		while(!IO.transmitters.spkm2.inputs.notification->isFresh()) {
			std::cout << "MP blocking for SPKM2 notification" << std::endl;
			ReceiveSingleMessage(true);
		}

		IO.transmitters.spkm2.inputs.notification->markAsUsed();

		if(IO.transmitters.spkm2.inputs.notification->Get() == lib::NACK) {
			BOOST_THROW_EXCEPTION(exception::nfe());
		}

		//sleep(1);
	}
#endif
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
