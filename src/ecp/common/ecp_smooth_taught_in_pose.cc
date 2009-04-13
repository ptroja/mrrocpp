#include <string.h>

#include "ecp/common/ecp_smooth_taught_in_pose.h"
#include "lib/impconst.h"

namespace mrrocpp {
namespace ecp {
namespace common {

ecp_smooth_taught_in_pose::ecp_smooth_taught_in_pose (void)
{}

ecp_smooth_taught_in_pose::ecp_smooth_taught_in_pose (POSE_SPECIFICATION at, double* vp, double* vk, double* vv, double* aa, double* c)
{
	arm_type=at;
	memcpy(v_p, vp, MAX_SERVOS_NR*sizeof(double));
	memcpy(v_k, vk, MAX_SERVOS_NR*sizeof(double));
	memcpy(v, vv, MAX_SERVOS_NR*sizeof(double));
	memcpy(a, aa, MAX_SERVOS_NR*sizeof(double));
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
}


} // namespace common
} // namespace ecp
} // namespace mrrocpp

