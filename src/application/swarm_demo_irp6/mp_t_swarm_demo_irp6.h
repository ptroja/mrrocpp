#if !defined(__MP_T_SWARM_DEMO_IRP6_H)
#define __MP_T_SWARM_DEMO_IRP6_H

#include "base/mp/mp_task.h"

namespace mrrocpp {
namespace mp {
namespace task {

/**
 * @defgroup swarm_demo swarm_demo
 * @ingroup application
 * A swarm demo application
 */

class swarm_demo : public task
{
protected:

public:

	/**
	 * Constructor.
	 */
	swarm_demo(lib::configurator &_config);
	/// utworzenie robotow
	void create_robots(void);
	void main_task_algorithm(void);

};

} // namespace task
} // namespace mp
} // namespace mrrocpp

#endif
