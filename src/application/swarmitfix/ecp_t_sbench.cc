#include "base/lib/sr/srlib.h"

#include "robot/sbench/ecp_r_sbench.h"

#include "ecp_t_sbench.h"
#include "ecp_g_sbench.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {
namespace task {

swarmitfix::swarmitfix(lib::configurator &_config) :
	task_t(_config),
	nextstateBuffer(*this, lib::commandBufferId)
{
	// Create the robot object
	ecp_m_robot = (boost::shared_ptr <robot_t>) new sbench::robot(*this);

	// Create task-dependent IO buffers
	notifyBuffer = (boost::shared_ptr<lib::agent::OutputBuffer<lib::notification_t> >)
			new lib::agent::OutputBuffer<lib::notification_t>(MP, ecp_m_robot->robot_name+lib::notifyBufferId);

	sr_ecp_msg->message("ecp sbench loaded");
}

void swarmitfix::main_task_algorithm(void)
{
	std::cerr << "sbench> swarmitfix::main_task_algorithm" << std::endl;

	if (0) {
		// Setup initial pin state
		lib::sbench::voltage_buffer pins;

		// Generator to execute the command
		generator::pin_config g_pin_setup(*this, pins);

		// Command the robot
		g_pin_setup.Move();
	}

	// Generator to change state of the pins
	generator::pin_config g_pin_config(*this, nextstateBuffer.access);

	// Loop execution coordinator's commands
	while(true) {
		// Wait for new coordinator's command
		while(!nextstateBuffer.isFresh()) {
			ReceiveSingleMessage(true);
		}

		try {
			// Mark command as used
			nextstateBuffer.markAsUsed();

			// Dispatch to the generator
			g_pin_config.Move();

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
} // namespace sbench

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new sbench::task::swarmitfix(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
