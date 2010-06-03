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

bool constant_velocity_profile::calculate_constant_velocity_profile(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, int i) {

	it->v_r[i] = it->s[i] / it->t;
	return true;
}

} // namespace velocity_profile_calculator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
