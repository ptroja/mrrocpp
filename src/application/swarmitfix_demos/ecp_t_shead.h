#if !defined(_ECP_T_SHEAD_SWARM_DEMO_SINGLE_AGENT_H)
#define _ECP_T_SHEAD_SWARM_DEMO_SINGLE_AGENT_H

#include <boost/shared_ptr.hpp>

#include "robot/shead/ecp_r_shead.h"
#include "base/ecp/ecp_task.h"
#include "ecp_g_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace task {

class swarmitfix : public common::task::_task <ecp::shead::robot>
{
protected:
	//! Move the robot the the specified pose
//	boost::shared_ptr <generator::shead_pose> g_pose;

//! Stop the robot in case of emergency
//	boost::shared_ptr <generator::shead_quickstop> g_quickstop;

	generator::sbench_transparent_generator* g_joint_epos_command;

public:
	//! Constructor
	swarmitfix(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void mp_2_ecp_next_state_string_handler(void);
};

} // namespace task
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
