/**
 * @file
 * @brief Contains definitions of the methods of trajectory_pose class.
 * @author rtulwin
 * @ingroup generators
 */

#include <cstring>

#include "base/lib/trajectory_pose/trajectory_pose.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

using namespace std;

trajectory_pose::trajectory_pose() {

}

trajectory_pose::trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
		const vector<double> & coordinates) {
	this->axes_num = coordinates.size();
	this->arm_type=arm_type;

	this->coordinates = coordinates;

	times = vector<double>(axes_num);
	k = vector<double>(axes_num);
	s = vector<double>(axes_num);
}

trajectory_pose::~trajectory_pose() {
}

} // namespace trajectory_pose
} // namespace common
} // namespace ecp
} // namespace mrrocpp
