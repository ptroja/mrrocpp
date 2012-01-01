#include <fstream>
#include <sstream>

#include <boost/foreach.hpp>
#include <boost/thread/thread_time.hpp>

#include "mp_t_swarmitfix.h"

#include "planner.h"

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include "serialization.h"

#include "base/lib/mrmath/homog_matrix.h"
#include "base/mp/mp_exceptions.h"

#include "robot/spkm/const_spkm1.h"
#include "robot/spkm/const_spkm2.h"
#include "robot/smb/const_smb1.h"
#include "robot/smb/const_smb2.h"
#include "robot/shead/const_shead1.h"
#include "robot/shead/const_shead2.h"

#include "base/lib/swarmtypes.h"
#include "base/mp/mp_robot.h"
#include "base/lib/ecp_ui_msg.h"

namespace mrrocpp {
namespace mp {
namespace task {

//! Check if PKM agent has completed all planned actions
bool isFinished(const Plan::PkmType::ItemConstIterator & it, const Plan & p)
{
	return (it == p.pkm().item().end());
}

//! Check if mobile base has completed all planned actions
bool isFinished(const Plan::MbaseType::ItemConstIterator & it, const Plan & p)
{
	return (it == p.mbase().item().end());
}

//! Fast-forward s-agent's plan item iterator until pointing next command
template<typename T>
void fastForward(T & it, int id, const Plan & p)
{
	while(!isFinished(it, p) && it->agent() != id) ++it;
}

//! Check if pointed s-agent's command matches time index
template<typename T>
bool indexMatches(const T & it, int ind, const Plan & p)
{
	return (!isFinished(it, p) && it->ind() == ind);
}

bool executeCommandItem(const Plan::PkmType::ItemType & pkmCmd, lib::agent::OutputBuffer<lib::spkm::next_state_t> * outBuf)
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

	// Setup command for the PKM
	lib::spkm::next_state_t cmd(lib::spkm::POSE_LIST);

	cmd.segments.push_back(hm);
//		cmd.segments.push_back(hm);

	std::cerr << "MP: spkm" << pkmCmd.agent() << " # of segments = " << cmd.segments.size() << std::endl;
	for(lib::spkm::next_state_t::segment_sequence_t::const_iterator it = cmd.segments.begin();
			it != cmd.segments.end();
			++it) {
		std::cerr << "pose\n" << it->goal_pose << std::endl;
		std::cerr << "motion type " << it->motion_type << std::endl;
		std::cerr << "duration " << it->duration << std::endl;
		std::cerr << "guarded_motion " << it->guarded_motion << std::endl;
	}

	// Send command if the output buffer is active
	if(outBuf) {
		outBuf->Send(cmd);
		return true;
	} else {
		return false;
	}
}

bool executeCommandItem(const Plan::MbaseType::ItemType & smbCmd, lib::agent::OutputBuffer<lib::smb::next_state_t> * outBuf)
{
	// Setup command for the mobile base
	lib::smb::next_state_t cmd(lib::smb::ACTION_LIST);

	std::cerr << "MP: smb" << smbCmd.agent() << " # of SMB segments = " << smbCmd.actions().item().size() << std::endl;

	// Iterate over action sequence
	BOOST_FOREACH(const Plan::MbaseType::ItemType::ActionsType::ItemType & it, smbCmd.actions().item())
	{
		std::cerr << "pin " << it.pin() << std::endl;
		std::cerr << "dPkmTheta " << it.dPkmTheta() << std::endl;

		// Setup single action
		lib::smb::action act;

		if(it.pin()) {
			act.setRotationPin(it.pin());
			act.setdThetaInd(it.dThetaInd());
		}
		// FIXME
		double normalizedPkmTheta = smbCmd.pkmTheta();
		while(normalizedPkmTheta > M_PI) normalizedPkmTheta -= 2*M_PI;
		while(normalizedPkmTheta < -M_PI) normalizedPkmTheta += 2*M_PI;
		act.setdPkmTheta(normalizedPkmTheta);

		// Append action to the command sequence
		cmd.actions.push_back(act);
	}

	// Send command if the output buffer is active
	if(outBuf) {
		outBuf->Send(cmd);
		return true;
	} else {
		return false;
	}
}

#if 0
//! Dummy generator for handling UI commands
class dummy : public mp::generator::generator {
public:
	//! Dummy implementation of required abstract method
	bool first_step() {
		throw std::logic_error("Generator not intended for execution");
	}

	//! Dummy implementation of required abstract method
	bool next_step() {
		throw std::logic_error("Generator not intended for execution");
	}

};
#endif

void swarmitfix::main_test_algorithm(void)
{
	sr_ecp_msg->message("swarm test started");

	// operator_reaction("Does it work?");

	// Create planner object
	// planner pp(config.value<std::string>("planpath"));

	sr_ecp_msg->message("plan OK");

	Plan * p = pp.getPlan();

	// Plan iterators
	Plan::PkmType::ItemConstIterator spkm1_it = p->pkm().item().begin();
	Plan::PkmType::ItemConstIterator spkm2_it = p->pkm().item().begin();
	Plan::MbaseType::ItemConstIterator smb1_it = p->mbase().item().begin();
	Plan::MbaseType::ItemConstIterator smb2_it = p->mbase().item().begin();

	// Setup iterators
	fastForward(spkm1_it, 1, *p);
	fastForward(spkm2_it, 2, *p);
	fastForward(smb1_it, 1, *p);
	fastForward(smb2_it, 2, *p);

	// Time index counter
	int ind = 0;

	// Setup index counter at the beginning of the plan
	BOOST_FOREACH(const Plan::PkmType::ItemType & it, p->pkm().item()) {
		if(ind > it.ind()) ind = it.ind();
	}
	BOOST_FOREACH(const Plan::MbaseType::ItemType & it, p->mbase().item()) {
		if(ind > it.ind()) ind = it.ind();
	}

	current_plan_status = ONGOING;

	for (; current_plan_status == ONGOING; ++ind) {
		// Diagnostic timestamp
		boost::system_time start_timestamp = boost::get_system_time();

		State * currentActionState;

//		std::cerr << "plan index = " << ind << "\t" <<
//			isFinished(spkm1_it, *p) <<
//			isFinished(spkm2_it, *p) <<
//			isFinished(smb1_it, *p) <<
//			isFinished(smb2_it, *p) << std::endl;

		// Execute command for spkm1
		if(indexMatches(spkm1_it, ind, *p)) {
			currentActionState = (State *) &(*spkm1_it);

			if(executeCommandItem(*spkm1_it++, IO.transmitters.spkm1.outputs.command.get()))
				current_workers_status[lib::spkm1::ROBOT_NAME] = WorkersStatus::BUSY;

			// Fast-forward upto next command
			fastForward(spkm1_it, 1, *p);
		}

		// Execute command for spkm2
		if(indexMatches(spkm2_it, ind, *p)) {
			currentActionState = (State *) &(*spkm2_it);

			if (executeCommandItem(*spkm2_it++, IO.transmitters.spkm2.outputs.command.get()))
				current_workers_status[lib::spkm2::ROBOT_NAME] = WorkersStatus::BUSY;

			// Fast-forward upto next command
			fastForward(spkm2_it, 2, *p);
		}

		// Execute command for smb1
		if(indexMatches(smb1_it, ind, *p)) {
			currentActionState = (State *) &(*smb1_it);

			if(executeCommandItem(*smb1_it++, IO.transmitters.smb1.outputs.command.get()))
				current_workers_status[lib::smb1::ROBOT_NAME] = WorkersStatus::BUSY;

			// Fast-forward upto next command
			fastForward(smb1_it, 1, *p);
		}

		// Execute command for smb2
		if(indexMatches(smb2_it, ind, *p)) {
			currentActionState = (State *) &(*smb2_it);

			if(executeCommandItem(*smb2_it++, IO.transmitters.smb2.outputs.command.get()))
				current_workers_status[lib::smb2::ROBOT_NAME] = WorkersStatus::BUSY;

			// Fast-forward upto next command
			fastForward(smb2_it, 2, *p);
		}
#if 0
		// Execute command for shead1
		if(indexMatches(shead1_it, ind, *p)) {
			currentActionState = (State *) &(*shead1_it);

			// TODO
			if(executeCommandItem(*shead1_it++))
				current_workers_status[lib::shead1::ROBOT_NAME] = WorkersStatus::BUSY;

			// Fast-forward upto next command
			fastForward(shead1_it, 1, *p);
		}

		// Execute command for shead2
		if(indexMatches(shead2_it, ind, *p)) {
			currentActionState = (State *) &(*shead2_it);

			// TODO
			if(executeCommandItem(*shead2_it++))
				current_workers_status[lib::shead2::ROBOT_NAME] = WorkersStatus::BUSY;

			// Fast-forward upto next command
			fastForward(shead2_it, 2, *p);
		}
#endif
		const bool record_timestamp = !current_workers_status.allIdle();

		while(!current_workers_status.allIdle()) {
			std::cout << "MP blocking for message" << std::endl;
			ReceiveSingleMessage(true);

			// Discard communication on control data channel
			BOOST_FOREACH(const common::robot_pair_t & robot_node, robot_m)
			{
				if (robot_node.second->reply.isFresh()) {

					robot_node.second->reply.markAsUsed();

					sr_ecp_msg->message("Unexpected communication on control data channel");
				}
			}

			if(IO.transmitters.smb1.inputs.notification.get() && IO.transmitters.smb1.inputs.notification->isFresh()) {
				IO.transmitters.smb1.inputs.notification->markAsUsed();
				current_workers_status[lib::smb1::ROBOT_NAME] = WorkersStatus::IDLE;

				if(IO.transmitters.smb1.inputs.notification->Get() == lib::NACK) {
					current_plan_status = FAILURE;
				}
			}

			if(IO.transmitters.smb2.inputs.notification.get() && IO.transmitters.smb2.inputs.notification->isFresh()) {
				IO.transmitters.smb2.inputs.notification->markAsUsed();
				current_workers_status[lib::smb2::ROBOT_NAME] = WorkersStatus::IDLE;

				if(IO.transmitters.smb2.inputs.notification->Get() == lib::NACK) {
					current_plan_status = FAILURE;
				}
			}

			if(IO.transmitters.spkm1.inputs.notification.get() && IO.transmitters.spkm1.inputs.notification->isFresh()) {
				IO.transmitters.spkm1.inputs.notification->markAsUsed();
				current_workers_status[lib::spkm1::ROBOT_NAME] = WorkersStatus::IDLE;

				if(IO.transmitters.spkm1.inputs.notification->Get() == lib::NACK) {
					current_plan_status = FAILURE;
				}
			}

			if(IO.transmitters.spkm2.inputs.notification.get() && IO.transmitters.spkm2.inputs.notification->isFresh()) {
				IO.transmitters.spkm2.inputs.notification->markAsUsed();
				current_workers_status[lib::spkm2::ROBOT_NAME] = WorkersStatus::IDLE;

				if(IO.transmitters.spkm2.inputs.notification->Get() == lib::NACK) {
					current_plan_status = FAILURE;
				}
			}
		}

		// Diagnostic timestamp
		if (record_timestamp) {
			// Get the stop timestamp
			boost::system_time stop_timestamp = boost::get_system_time();

			// Calculate command duration
			boost::posix_time::time_duration td = stop_timestamp - start_timestamp;

			std::cout << "Command duration in [ms] is " << td.total_milliseconds() << std::endl;
			currentActionState->state_reached_in_time().set(td.total_milliseconds()/1000.0);

//			// Wait for trigger
//			while(!(ui_pulse.isFresh() && ui_pulse.Get() == MP_TRIGGER)) {
//				// TODO: handle PAUSE/RESUME/STOP commands as well
//				if(ui_pulse.isFresh()) ui_pulse.markAsUsed();
//				ReceiveSingleMessage(true);
//			}
//
//			ui_pulse.markAsUsed();

			{
				// create archive
				std::stringstream ofs;
				{
					boost::archive::xml_oarchive oa(ofs);

					// serialize data into XML
					oa << boost::serialization::make_nvp("item", p->pkm().item().front());
				}

				// Request
				lib::ECP_message ecp_to_ui_msg;

				// Setup plan item
				ecp_to_ui_msg.ecp_message = lib::PLAN_STEP_MODE;
				ecp_to_ui_msg.plan_item = ofs.str();

				// Reply
				lib::UI_reply ui_to_ecp_rep;

				if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

					uint64_t e = errno;
					perror("ecp operator_reaction(): Send() to UI failed");
					sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
					BOOST_THROW_EXCEPTION(exception::se());
				}
			}
		}

		// If all iterators are at the end
		if(
				isFinished(spkm1_it, *p) &&
				isFinished(spkm2_it, *p) &&
				isFinished(smb1_it, *p) &&
				isFinished(smb2_it, *p)
				)
		{
			// Then finish
			break;
		}
	}

//	xml_schema::namespace_infomap map;
//	map[""].schema = "people.xsd";

	// Serialize to a file.
	//
	{
		std::cout << "Serialize to a file." << std::endl;
		std::ofstream ofs ("result.xml");
		plan(ofs, *p);
	}


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
