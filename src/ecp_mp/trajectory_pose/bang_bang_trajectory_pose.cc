/**
 * \file bang_bang_trajectory_pose.cc
 * \brief bang_bang_trajectory_pose class and its methods
 *
 * Contains bodies of the methods of bang_bang_trajectory_pose class.
 */

#include <string.h>

#include "ecp_mp/trajectory_pose/bang_bang_trajectory_pose.h"

namespace mrrocpp {
namespace ecp_mp {
namespace common {
namespace trajectory_pose {

bang_bang_trajectory_pose::bang_bang_trajectory_pose (void) {
}

bang_bang_trajectory_pose::bang_bang_trajectory_pose (lib::ECP_POSE_SPECIFICATION arm_type,
		const vector<double> & coordinates,
		const vector<double> & v,
		const vector<double> & a,
		const int axes_num) :
		trajectory_pose (arm_type, coordinates, axes_num) {

	this->v = v;
	this->a = a;

	v_p = vector<double>();
	v_k = vector<double>();
	v_max = vector<double>();
	przysp = vector<double>();
	jedn = vector<double>();
	s_jedn = vector<double>();
	s_przysp = vector<double>();
	start_position = vector<double>();
	k = vector<double>();
	a_r = vector<double>();
	v_r = vector<double>();
	model = vector<int>();
	s = vector<double>();
}

bang_bang_trajectory_pose::~bang_bang_trajectory_pose() {

}

} // namespace trajectory_pose
} // namespace common
} // namespace ecp
} // namespace mrrocpp
