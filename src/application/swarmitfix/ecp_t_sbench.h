#if !defined(_ECP_T_SBENCH_SWARMITFIX_H)
#define _ECP_T_SBENCH_SWARMITFIX_H

#include <boost/shared_ptr.hpp>

#include "robot/sbench/ecp_r_sbench.h"
#include "base/ecp/ecp_task.h"

#include "base/lib/agent/OutputBuffer.h"
#include "base/lib/swarmtypes.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {
namespace task {

class swarmitfix : public common::task::_task<ecp::sbench::robot>
{
public:
	//! Constructor
	swarmitfix(lib::configurator &_config);

	// methods for ECP template to redefine in concrete classes
	void main_task_algorithm(void);

protected:
	/**
	 * Input buffer for MP commands
	 */
	lib::agent::InputBuffer<lib::sbench::voltage_buffer> nextstateBuffer;

	/**
	 * Output buffer for MP notifications
	 */
	boost::shared_ptr<lib::agent::OutputBuffer<lib::notification_t> > notifyBuffer;
};

} // namespace task
} // namespace irp6p
} // namespace ecp
} // namespace mrrocpp

#endif
