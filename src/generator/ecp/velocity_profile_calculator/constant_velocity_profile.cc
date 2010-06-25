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

	//if (it->s[i] != NULL && it->t != NULL) {
		it->v_r[i] = it->s[i] / it->t;
		return true;
	//} else {
	//	return false;
	//}
}

bool constant_velocity_profile::calculate_constant_velocity_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	bool trueFlag = true;

	for (int i; i < it->axes_num; i++) {
		if (calculate_constant_velocity(it, i) == false) {
			printf("nieudane calculate constant velocity pose na elemencie %d\n", i);
			flushall();
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool constant_velocity_profile::calculate_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i) {

	//if (it->v_r[i] != NULL && it->s[i] != NULL) {
		it->times[i] = it->s[i] / it->v_r[i];
		return true;
	//} else {
		//return false;
	//}
}

bool constant_velocity_profile::calculate_time_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	bool trueFlag = true;

	for (int i; i < it->axes_num; i++) {
		if (calculate_time(it, i) == false) {
			printf("nieudane calculate time pose na elemencie %d\n", i);
			flushall();
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool constant_velocity_profile::calculate_pose_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {
	if (it->times.size() == it->axes_num) {
		it->t = *max_element(it->times.begin(), it->times.end());
		return true;
	} else {
		printf("nieudane calculate pose time\n");
		flushall();
		return false;
	}
}

bool constant_velocity_profile::calculate_absolute_distance_direction_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	printf("############## calculate absolute distance direction pose ##############\n");
	printf("coordinates.size(): %d \t axes_num: %d \t start_position.size(): %d \n", it->coordinates.size(), it->axes_num, it->start_position.size());

	if (it->coordinates.size() < it->axes_num || it->start_position.size() < it->axes_num) {
		printf("nieudane calculate absolute distance direction pose\n");
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
