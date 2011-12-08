#include "base/lib/sr/srlib.h"

#include "robot/smb/ecp_r_smb1.h"
#include "robot/smb/ecp_r_smb2.h"

#include "ecp_t_smb.h"
#include "ecp_g_smb.h"
#include "ecp_mp_g_smb.h"

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

	// TODO: Create the generators

	sr_ecp_msg->message("ecp smb loaded");
}

void swarmitfix::main_task_algorithm(void)
{
	std::cerr << "> swarmitfix::main_task_algorithm" << std::endl;

	// Loop execution coordinator's commands
	while(true) {
		// Wait for new coordinator's command
		while(!nextstateBuffer.isFresh()) {
			ReceiveSingleMessage(true);
		}

		try {
			// Mark command as used
			nextstateBuffer.markAsUsed();

			// Dispatch to selected generator
			switch(nextstateBuffer.Get().variant) {
				case lib::smb::ACTION_LIST:
					//g_action->Move();
					break;
				default:
					//g_quickstop->Move();
					break;
			}

		} catch (std::exception & e) {
			// Report problem and re-throw exception to the process shell
			notifyBuffer->Send(lib::NACK);
			throw;
		}

		// Reply with acknowledgment
		notifyBuffer->Send(lib::ACK);
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
