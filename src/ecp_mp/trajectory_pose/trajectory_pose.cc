/**
 * \file trajectory_pose.cc
 * \brief rajectory_pose class and its methods
 *
 * Contains bodies of the methods of trajectory_pose class.
 */

#include <string.h>

#include "ecp_mp/trajectory_pose/trajectory_pose.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

trajectory_pose::trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
		const vector<double> & coordinates, const int axes_num) {
	this->axes_num = axes_num;
	this->arm_type=arm_type;

	this->coordinates = coordinates;
}

trajectory_pose::~trajectory_pose() {
}

} // namespace trajectory_pose
} // namespace common
} // namespace ecp
} // namespace mrrocpp
