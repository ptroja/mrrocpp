#include "base/lib/sr/srlib.h"

#include "robot/spkm/ecp_r_spkm1.h"
#include "robot/spkm/ecp_r_spkm2.h"

#include "ecp_t_spkm.h"
#include "ecp_g_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace task {

swarmitfix::swarmitfix(lib::configurator &_config) :
	task_t(_config),
	nextstateBuffer(*this, lib::commandBufferId)
{
	// Create the robot object
	if (config.robot_name == lib::spkm1::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new spkm1::robot(*this);
	} else if (config.robot_name == lib::spkm2::ROBOT_NAME) {
		ecp_m_robot = (boost::shared_ptr <robot_t>) new spkm2::robot(*this);
	} else {
		throw std::runtime_error(config.robot_name + ": unknown robot");
	}

	// Create task-dependent IO buffers
	notifyBuffer = (boost::shared_ptr<lib::agent::OutputBuffer<lib::notification_t> >)
			new lib::agent::OutputBuffer<lib::notification_t>(MP, ecp_m_robot->robot_name+lib::notifyBufferId);

	sr_ecp_msg->message("ecp spkm loaded");
}

void swarmitfix::main_task_algorithm(void)
{
	std::cerr << "spkm> swarmitfix::main_task_algorithm" << std::endl;

	if (0) {
		// Start PKM pose (also known as "neutral")
		lib::Homog_matrix hm = lib::Xyz_Euler_Zyz_vector(
				0.15, 0, 0.405, 0, -1.045, 0
				);

		// Setup single motion segment
		lib::spkm::segment_t segment(hm);

		// Generator for motion execution
		generator::spkm_pose g_pose(*this, segment);

		// Move the robot the the specified pose
		g_pose.Move();
	}

	//! Move the robot the the specified pose
	generator::spkm_pose g_pose(*this, nextstateBuffer.access.segment);

	//! Stop the robot in case of emergency
	generator::spkm_quickstop g_quickstop(*this);

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
				case lib::spkm::GOAL_POSE:
					g_pose.Move();
					break;
				default:
					g_quickstop.Move();
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

}
} // namespace spkm

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new spkm::task::swarmitfix(_config);
}

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp
