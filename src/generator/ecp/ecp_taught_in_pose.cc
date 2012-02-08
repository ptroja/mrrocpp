/*!
 * @file
 * @brief File contains ecp_taught_in_pose definition
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 *
 * @ingroup ecp
 */

#include <cstring>

#include "ecp_taught_in_pose.h"

namespace mrrocpp {
namespace ecp {
namespace common {

ecp_taught_in_pose::ecp_taught_in_pose(void)
{
}

ecp_taught_in_pose::ecp_taught_in_pose(lib::ECP_POSE_SPECIFICATION at, double mt, const double c[lib::MAX_SERVOS_NR], int e_info) // by Y
:
	arm_type(at), motion_time(mt), extra_info(e_info)
{
	memcpy(coordinates, c, lib::MAX_SERVOS_NR * sizeof(double));
} // end: ecp_taught_in_pose::ecp_taught_in_pose


} // namespace common
} // namespace ecp
} // namespace mrrocpp
