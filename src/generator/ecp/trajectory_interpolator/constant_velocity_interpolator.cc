/*
 * constant_velocity_interpolator.cpp
 *
 *  Created on: Jun 3, 2010
 *      Author: rtulwin
 */

#include "constant_velocity_interpolator.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {
namespace trajectory_interpolator {

constant_velocity_interpolator::constant_velocity_interpolator() {
	// TODO Auto-generated constructor stub

}

constant_velocity_interpolator::~constant_velocity_interpolator() {
	// TODO Auto-generated destructor stub
}

bool constant_velocity_interpolator::interpolate_relative(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, vector<vector<double> >::iterator & cit) {

	//next_position = k * (s_przysp + ((node_counter - przysp) * tk) * v_r);
	return true;
}

bool constant_velocity_interpolator::interpolate_absolute(vector<ecp_mp::common::trajectory_pose::constant_velocity_trajectory_pose>::iterator & it, vector<vector<double> >::iterator & cit) {

	//next_position = it->start_position + k * (s_przysp + ((node_counter - przysp) * tk) * v_r);
	return true;
}

} // namespace trajectory_interpolator
} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
