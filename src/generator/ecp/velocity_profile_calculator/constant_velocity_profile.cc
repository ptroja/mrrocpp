/*
 * constant_velocity_profile.cc
 *
 *  Created on: May 4, 2010
 *      Author: rtulwin
 */

#include "constant_velocity_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

constant_velocity_profile::constant_velocity_profile() {
	// TODO Auto-generated constructor stub

}

constant_velocity_profile::~constant_velocity_profile() {
	// TODO Auto-generated destructor stub
}

bool constant_velocity_profile::calculate_constant_velocity(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i) {

	if (it->s[i] != NULL && it->t != NULL) {
		it->v_r[i] = it->s[i] / it->t;
		return true;
	} else {
		return false;
	}
}

bool constant_velocity_profile::calculate_constant_velocity_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	bool trueFlag = true;

	for (int i; i < it->axes_num; i++) {
		if (calculate_constant_velocity(it, i) == false) {
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool constant_velocity_profile::calculate_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i) {

	if (it->v_r[i] != NULL && it->s[i] != NULL) {
		it->times[i] = it->s[i] / it->v_r[i];
		return true;
	} else {
		return false;
	}
}

bool constant_velocity_profile::calculate_time_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	for (int i; i < it->axes_num; i++) {
		if (calculate_time(it, i) == false) {
			return false;
		}
	}

	return true;
}

bool constant_velocity_profile::calculate_pose_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {
	if (it->times.size() == it->axes_num) {
		it->t = *max_element(it->times.begin(), it->times.end());
		return true;
	} else {
		return false;
	}
}

bool constant_velocity_profile::calculate_distance_direction_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	for (int i = 0; i < it->axes_num; i++) {
		it->s[i] = abs(it->coordinates[i] - it->start_position[i]);
		if (it->coordinates[i] - it->start_position[i] >= 0) {
			it->k[i] = 1;
		} else {
			it->k[i] = -1;
		}
	}

	return true;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
