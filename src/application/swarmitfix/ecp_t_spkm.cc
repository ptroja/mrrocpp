#include "base/lib/sr/srlib.h"

#include "robot/spkm/ecp_r_spkm1.h"
#include "robot/spkm/ecp_r_spkm2.h"

#include "ecp_t_spkm.h"
#include "ecp_g_spkm.h"
#include "ecp_mp_g_spkm.h"

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
	notifyBuffer = (boost::shared_ptr<OutputBuffer<lib::notification_t> >)
			new OutputBuffer<lib::notification_t>(MP, ecp_m_robot->robot_name+lib::notifyBufferId);

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

		// Setup single motion sequence
		lib::spkm::next_state_t::segment_sequence_t sequence;

		// Insert single motion segment
		sequence.push_back(hm);

		// Generator for motion execution
		generator::spkm_pose g_pose(*this, sequence);

		// Move the robot the the specified pose
		g_pose.Move();
	}

	//! Move the robot the the specified pose
	generator::spkm_pose g_pose(*this, nextstateBuffer.access.segments);

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
				case lib::spkm::POSE_LIST:
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
