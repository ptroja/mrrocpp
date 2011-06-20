/**
 * @file
 * @brief Contains definitions of the methods of constant_velocity_profile class.
 * @author rtulwin
 * @ingroup generators
 */

#include "constant_velocity_profile.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace velocity_profile_calculator {

using namespace std;

constant_velocity_profile::constant_velocity_profile() {
	// TODO Auto-generated constructor stub

}

constant_velocity_profile::~constant_velocity_profile() {
	// TODO Auto-generated destructor stub
}

bool constant_velocity_profile::calculate_constant_velocity(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i) {

	if (eq(it->s[i], 0.0) || eq(it->t, 0.0)) {//if the distance to be covered equals to 0 or pose time equal to 0 (no motion in a pose)
		it->v_r[i] = 0;
	} else {//normal calculation
		it->v_r[i] = it->s[i] / it->t;
	}
	return true;
}

bool constant_velocity_profile::calculate_constant_velocity_pose(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it) {

	bool trueFlag = true;

	for (int i = 0; i < it->axes_num; i++) {
		if (calculate_constant_velocity(it, i) == false) {
			trueFlag = false;
		}
	}

	return trueFlag;
}

bool constant_velocity_profile::calculate_time(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i) {

	if (eq(it->s[i], 0.0) || eq(it->v_r[i], 0.0)) {//if distance to be covered or maximal velocity equal to 0
		it->times[i] = 0;
	} else {//normal calculation
		it->times[i] = it->s[i] / it->v_r[i];
	}
	return true;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
