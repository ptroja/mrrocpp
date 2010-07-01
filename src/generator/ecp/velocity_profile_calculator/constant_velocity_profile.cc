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

	if (it->s[i] == 0 || it->t == 0) {//if the distance to be covered equals to 0 or pose time equal to 0 (no motion in a pose)
		it->v_r[i] = 0;
	} else {//normal calculation
		it->v_r[i] = it->s[i] / it->t;
	}
	return true;
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

	if (it->s[i] == 0 || it->v_r[i] == 0) {//if distance to be covered or maximal velocity equal to 0
		it->times[i] = 0;
	} else {//normal calculation
		it->times[i] = it->s[i] / it->v_r[i];
	}
	return true;
}

bool constant_velocity_profile::calculate_time_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	bool trueFlag = true;

	for (int i; i < it->axes_num; i++) {
		if (calculate_time(it, i) == false) {
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool constant_velocity_profile::calculate_pose_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, const double & mc) {
	if (it->times.size() == it->axes_num) {
		double t_max = *max_element(it->times.begin(), it->times.end());

		if (t_max == 0) {
			it->t = 0;
			return true;
		}

		if (ceil(t_max / mc) * mc != t_max) { //extend the pose time to be the multiplicity of the macrostep time
			t_max = ceil(t_max / mc);
			t_max = t_max * mc;
			it->t = t_max;
		}

		return true;
	} else {
		return false;
	}
}

bool constant_velocity_profile::calculate_absolute_distance_direction_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	if (it->coordinates.size() < it->axes_num || it->start_position.size() < it->axes_num) {
		return false;
	}

	it->s.clear();
	it->k.clear();
	for (int i = 0; i < it->axes_num; i++) {
		it->s.push_back(abs(it->coordinates[i] - it->start_position[i]));
		if (it->coordinates[i] - it->start_position[i] >= 0) {
			it->k.push_back(1);
		} else {
			it->k.push_back(-1);
		}
	}

	return true;
}

bool constant_velocity_profile::calculate_relative_distance_direction_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	if (it->coordinates.size() < it->axes_num) {
		return false;
	}

	it->s.clear();
	it->k.clear();
	for (int i = 0; i < it->axes_num; i++) {
		it->s.push_back(it->coordinates[i]);
		if (it->coordinates[i] >= 0) {
			it->k.push_back(1);
		} else {
			it->k.push_back(-1);
		}
	}

	return true;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
