#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "robot/shead/ecp_r_shead.h"
#include "robot/shead/ecp_r_shead1.h"
#include "robot/shead/ecp_r_shead2.h"
#include "ecp_g_shead.h"
#include "ecp_t_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace task {

// KONSTRUKTORY
swarmitfix::swarmitfix(lib::configurator &_config) :
	task_t(_config),
	nextstateBuffer(*this, lib::commandBufferId)
{
	// Create the robot object
	if (config.robot_name == lib::shead1::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new shead1::robot(*this);
	} else if (config.robot_name == lib::shead2::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new shead2::robot(*this);
	} else {
		throw std::runtime_error(config.robot_name + ": unknown robot");
	}

	// Create task-dependent IO buffers
	notifyBuffer = (boost::shared_ptr<lib::agent::OutputBuffer<lib::notification_t> >)
			new lib::agent::OutputBuffer<lib::notification_t>(MP, ecp_m_robot->robot_name+lib::notifyBufferId);

	sr_ecp_msg->message("ecp shead loaded");
}

void swarmitfix::main_task_algorithm(void)
{
	std::cerr << "shead> swarmitfix::main_task_algorithm" << std::endl;

	if (0) {
		// Setup initial off state
		lib::shead::next_state::control_t cmd;

		cmd.solidify = lib::shead::SOLIDIFICATION_OFF;
		cmd.vacuum = lib::shead::VACUUM_OFF;

		// Generator to execute the command
		generator::control g_control(*this, cmd);

		// Command the robot
		g_control.Move();
	}

	// Generators to execute coordinator's commands
	generator::control g_control(*this, nextstateBuffer.access.control);
	generator::rotate g_rotate(*this, nextstateBuffer.access.pose);

	// Loop execution coordinator's commands
	while(true) {
		// Wait for new coordinator's command
		while(!nextstateBuffer.isFresh()) {
			ReceiveSingleMessage(true);
		}

		try {
			switch(nextstateBuffer.access.command) {
				case lib::shead::next_state::CONTROL:
					g_control.Move();
					break;
				case lib::shead::next_state::ROTATE:
					g_control.Move();
					break;
				default:
					// TODO: execute quickstop command
					throw std::runtime_error("Quickstop not implemented");
					break;
			}
			// Mark command as used
			nextstateBuffer.markAsUsed();

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

}
} // namespace shead

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new shead::task::swarmitfix(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
