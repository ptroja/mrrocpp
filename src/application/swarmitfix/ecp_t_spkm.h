#if !defined(_ECP_T_SPKM_SWARMITFIX_H)
#define _ECP_T_SPKM_SWARMITFIX_H

#include <boost/shared_ptr.hpp>

#include "robot/spkm/ecp_r_spkm.h"
#include "base/ecp/ecp_task.h"
#include "ecp_g_spkm.h"

#include "base/lib/agent/OutputBuffer.h"
#include "base/lib/swarmtypes.h"

#include <iostream>

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace task {

class swarmitfix : public common::task::_task<ecp::spkm::robot>
{
public:
	//! Constructor
	swarmitfix(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);

protected:
	//! Move the robot the the specified pose
	boost::shared_ptr<generator::spkm_pose> g_pose;

	//! Stop the robot in case of emergency
	boost::shared_ptr<generator::spkm_quickstop> g_quickstop;

	/**
	 * Input buffer for MP commands
	 */
	InputBuffer<lib::spkm::next_state_t> nextstateBuffer;

	/**
	 * Output buffer for MP notifications
	 */
	boost::shared_ptr<OutputBuffer<lib::notification_t> > notifyBuffer;
};

} // namespace task
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
