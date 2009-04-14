#include <string.h>

#include "ecp/common/ecp_smooth2_taught_in_pose.h"
#include "lib/impconst.h"

namespace mrrocpp {
namespace ecp {
namespace common {

ecp_smooth2_taught_in_pose::ecp_smooth2_taught_in_pose (void)
{}

ecp_smooth2_taught_in_pose::ecp_smooth2_taught_in_pose (lib::POSE_SPECIFICATION at, double* vv, double* aa, double* c)
{
	arm_type=at;
	memcpy(v, vv, MAX_SERVOS_NR*sizeof(double));
	memcpy(a, aa, MAX_SERVOS_NR*sizeof(double));
	memcpy(coordinates, c, MAX_SERVOS_NR*sizeof(double));
}

} // namespace common
} // namespace ecp
} // namespace mrrocpp
