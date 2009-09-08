#include <string.h>

#include "ecp_mp/smooth_trajectory_pose.h"
#include "lib/impconst.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {

smooth_trajectory_pose::smooth_trajectory_pose (void)
{}

smooth_trajectory_pose::smooth_trajectory_pose (lib::POSE_SPECIFICATION at,
		const double* c,
		const double* vv,
		const double* aa,
		const double* vp,
		const double* vk)
{
	arm_type=at;
	memcpy(v_p, vp, MAX_SERVOS_NR*sizeof(double));
	memcpy(v_k, vk, MAX_SERVOS_NR*sizeof(double));
	memcpy(v, vv, MAX_SERVOS_NR*sizeof(double));
	memcpy(a, aa, MAX_SERVOS_NR*sizeof(double));
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
}


} // namespace common
} // namespace ecp_mp
} // namespace mrrocpp

