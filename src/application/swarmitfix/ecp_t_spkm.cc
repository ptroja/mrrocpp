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

	// Create the generators
	g_pose = (boost::shared_ptr <generator::spkm_pose>) new generator::spkm_pose(*this, nextstateBuffer.access.segments);
	g_quickstop = (boost::shared_ptr <generator::spkm_quickstop>) new generator::spkm_quickstop(*this);

	sr_ecp_msg->message("ecp spkm loaded");
}

void swarmitfix::main_task_algorithm(void)
{
	std::cerr << "> swarmitfix::main_task_algorithm" << std::endl;

	while(true) {
		while(!nextstateBuffer.isFresh()) {
			ReceiveSingleMessage(true);
		}

		switch(nextstateBuffer.Get().variant) {
			case lib::spkm::POSE_LIST:
				g_pose->Move();
				break;
			default:
				g_quickstop->Move();
				break;
		}

		nextstateBuffer.markAsUsed();
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
