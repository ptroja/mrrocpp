#include <boost/foreach.hpp>

#include "base/lib/sr/srlib.h"

#include "robot/smb/ecp_r_smb1.h"
#include "robot/smb/ecp_r_smb2.h"

#include "ecp_t_smb.h"
#include "ecp_g_smb.h"
#include "ecp_mp_g_smb.h"
#include "robot/smb/dp_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace task {

// KONSTRUKTORY
swarmitfix::swarmitfix(lib::configurator &_config) :
	task_t(_config),
	nextstateBuffer(*this, lib::commandBufferId)
{
	// Create the robot object
	if (config.robot_name == lib::smb1::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new smb1::robot(*this);
	} else if (config.robot_name == lib::smb2::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new smb2::robot(*this);
	} else {
		throw std::runtime_error(config.robot_name + ": unknown robot");
	}

	// Create task-dependent IO buffers
	notifyBuffer = (boost::shared_ptr<OutputBuffer<lib::notification_t> >)
			new OutputBuffer<lib::notification_t>(MP, ecp_m_robot->robot_name+lib::notifyBufferId);

	sr_ecp_msg->message("ecp smb loaded");
}

void swarmitfix::main_task_algorithm(void)
{
	std::cerr << "smb> swarmitfix::main_task_algorithm" << std::endl;

	{
		// Execute motion generator (defaults to OUT)
		generator::stand_up gen(*this);
		gen.Move();
	}

	// Loop execution coordinator's commands
	while(true) {
		// Wait for new coordinator's command
		while(!nextstateBuffer.isFresh()) {
			ReceiveSingleMessage(true);

			// TODO: handle commands at control channel
		}

		try {
			// Mark command as used
			nextstateBuffer.markAsUsed();

			// Dispatch to selected generator
			switch(nextstateBuffer.Get().variant) {
				case lib::smb::ACTION_LIST:
					execute_actions(nextstateBuffer.Get().actions);
					break;
				default:
				{
					generator::quickstop gen(*this);
					gen.Move();
				}
					break;
			}

		} catch (const std::exception & e) {
			// Report problem...
			notifyBuffer->Send(lib::NACK);

			// And DO NOT re-throw exception to the process shell
			// throw;
		}

		// Reply with acknowledgment
		notifyBuffer->Send(lib::ACK);
	}
}

void swarmitfix::execute_actions(const lib::smb::next_state_t::action_sequence_t & actions)
{
	BOOST_FOREACH(const lib::smb::action & act, actions)
	{
		// Legs up
		if(act.getRotationPin())
		{
			// Setup EDP command
			lib::smb::festo_command_td cmd;

			// All IN...
			cmd.leg[0] = lib::smb::IN;
			cmd.leg[1] = lib::smb::IN;
			cmd.leg[2] = lib::smb::IN;

			// and only one OUT.
			cmd.leg[act.getRotationPin()-1] = lib::smb::OUT;

			// Execute motion generator
			generator::stand_up gen(*this, cmd);
			gen.Move();
		}

		// Rotate
		{
			// Setup EDP command
			lib::smb::motor_command cmd;

			// Copy parameters
			//cmd.base_vs_bench_rotation = act.getdThetaInd();
			cmd.pkm_vs_base_rotation = act.getdPkmTheta();
			cmd.estimated_time = act.getDuration();

			// Execute motion generator
			generator::rotate gen(*this, cmd);
			gen.Move();
		}

		// Legs down
		if(act.getRotationPin())
		{
			// Execute motion generator (defaults to OUT)
			generator::stand_up gen(*this);
			gen.Move();
		}
	}
}

}
} // namespace smb

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new smb::task::swarmitfix(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
