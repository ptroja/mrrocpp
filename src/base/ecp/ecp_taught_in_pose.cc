#include <cstring>

#include "base/ecp/ecp_taught_in_pose.h"
#include "base/lib/impconst.h"

namespace mrrocpp {
namespace ecp {
namespace common {

ecp_taught_in_pose::ecp_taught_in_pose (void)
{
}

ecp_taught_in_pose::ecp_taught_in_pose (lib::ECP_POSE_SPECIFICATION at, double mt, const double c[MAX_SERVOS_NR], int e_info) // by Y
		:
		arm_type(at),
		motion_time(mt),
		extra_info(e_info)
{
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
} // end: ecp_taught_in_pose::ecp_taught_in_pose


} // namespace common
} // namespace ecp
} // namespace mrrocpp
