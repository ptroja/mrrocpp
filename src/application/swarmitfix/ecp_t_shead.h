#if !defined(_ECP_T_SHEAD_SWARMITFIX_H)
#define _ECP_T_SHEAD_SWARMITFIX_H

#include <boost/shared_ptr.hpp>

#include "robot/shead/ecp_r_shead.h"
#include "base/ecp/ecp_task.h"
#include "robot/shead/dp_shead.h"

#include "base/lib/agent/OutputBuffer.h"
#include "base/lib/agent/InputBuffer.h"
#include "base/lib/swarmtypes.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace task {

class swarmitfix: public common::task::_task<ecp::shead::robot>
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
	lib::agent::InputBuffer<lib::shead::next_state> nextstateBuffer;

	/**
	 * Output buffer for MP notifications
	 */
	boost::shared_ptr<lib::agent::OutputBuffer<lib::notification_t> > notifyBuffer;
};

}
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

#endif
