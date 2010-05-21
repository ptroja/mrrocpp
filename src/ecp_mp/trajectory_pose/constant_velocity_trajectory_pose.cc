/**
 * \file constant_velocity_trajectory_pose.cc
 * \brief constant_velocity_trajectory_pose class and its methods
 *
 * Contains bodies of the methods of bang_bang_trajectory_pose class.
 */

#include <string.h>

#include "ecp_mp/trajectory_pose/constant_velocity_trajectory_pose.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

constant_velocity_trajectory_pose::constant_velocity_trajectory_pose (void) {
}

constant_velocity_trajectory_pose::constant_velocity_trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
		const vector<double> & coordinates,
		const vector<double> & v,
		const int axes_num) :
		trajectory_pose (arm_type, coordinates, axes_num) {

	this->v = v;

	v_max = vector<double>();
	start_position = vector<double>();
	v_r = vector<double>();
	s = vector<double>();
}

constant_velocity_trajectory_pose::~constant_velocity_trajectory_pose() {

}

} // namespace trajectory_pose
} // namespace common
} // namespace ecp
} // namespace mrrocpp
