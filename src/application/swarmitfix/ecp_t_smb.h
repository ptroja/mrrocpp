#if !defined(_ECP_T_SMB_SWARMITFIX_H)
#define _ECP_T_SMB_SWARMITFIX_H

#include <boost/shared_ptr.hpp>

#include "robot/smb/ecp_r_smb.h"
#include "base/ecp/ecp_task.h"
#include "ecp_g_smb.h"

#include "base/lib/agent/OutputBuffer.h"
#include "base/lib/swarmtypes.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace task {

class swarmitfix : public common::task::_task<ecp::smb::robot>
{
public:
	//! Constructor
	swarmitfix(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);

private:
	/**
	 * Input buffer for MP commands
	 */
	InputBuffer<lib::smb::next_state_t> nextstateBuffer;

	/**
	 * Output buffer for MP notifications
	 */
	boost::shared_ptr<OutputBuffer<lib::notification_t> > notifyBuffer;

	//! Execute translocate command action
	void execute_actions(const lib::smb::next_state_t::action_sequence_t & actions);
};

}
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
