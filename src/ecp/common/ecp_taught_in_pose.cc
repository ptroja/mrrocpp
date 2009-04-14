#include <string.h>

#include "ecp/common/ecp_taught_in_pose.h"
#include "lib/impconst.h"

namespace mrrocpp {
namespace ecp {
namespace common {

ecp_taught_in_pose::ecp_taught_in_pose (void)
{}

ecp_taught_in_pose::ecp_taught_in_pose (lib::POSE_SPECIFICATION at, double mt, double* c)
		:
		arm_type(at),
		motion_time(mt)
{
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
} // end: ecp_taught_in_pose::ecp_taught_in_pose

ecp_taught_in_pose::ecp_taught_in_pose (lib::POSE_SPECIFICATION at, double mt, int e_info, double* c) // by Y
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
