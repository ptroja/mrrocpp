/**
 * \file bang_bang_trajectory_pose.cc
 * \brief bang_bang_trajectory_pose class and its methods
 *
 * Contains bodies of the methods of bang_bang_trajectory_pose class.
 */

#include <string.h>

#include "ecp_mp/bang_bang_trajectory_pose.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {

bang_bang_trajectory_pose::bang_bang_trajectory_pose (void)
{
}

bang_bang_trajectory_pose::bang_bang_trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
		const vector<double> & coordinates,
		const vector<double> & v,
		const vector<double> & a,
		const int axes_num)
{

	this->axes_num = axes_num;
	this->arm_type=arm_type;

	this->coordinates = coordinates;
	this->v = v;
	this->a = a;
}

bang_bang_trajectory_pose::~bang_bang_trajectory_pose() {

}

} // namespace common
} // namespace ecp
} // namespace mrrocpp
