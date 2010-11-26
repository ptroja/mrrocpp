#if !defined(_ECP_T_SWARM_DEMO_H)
#define _ECP_T_SWARM_DEMO_H

#include <cstdio>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "robot/irp6ot_m/ecp_r_irp6ot_m.h"
#include "robot/irp6p_m/ecp_r_irp6p_m.h"

#include "ecp_mp_t_swarm_demo.h"
#include "ecp_st_smooth_joint.h"
#include "ecp_st_smooth_angle_axis.h"

#include "base/ecp/ecp_task.h"
#include "application/swarm_demo/ecp_mp_st_smooth_joint.h"
#include "application/swarm_demo/ecp_mp_st_smooth_angle_axis.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace task {

class swarm_demo : public common::task::task
{
protected:

public:
	/**
	 * Constructor.
	 */
	swarm_demo(lib::configurator &_config);

};

}
} // namespace common
} // namespace ecp
} // namespace mrrocpp

#endif
