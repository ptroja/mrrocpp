#include <string.h>

#include "ecp_mp/trajectory_pose.h"
#include "lib/impconst.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {

trajectory_pose::trajectory_pose (void)
{}

trajectory_pose::trajectory_pose (lib::POSE_SPECIFICATION at,
		const double* c,
		const double* vv,
		const double* aa)
{
	arm_type=at;
	memcpy(v, vv, MAX_SERVOS_NR*sizeof(double));
	memcpy(a, aa, MAX_SERVOS_NR*sizeof(double));
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
}


} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

