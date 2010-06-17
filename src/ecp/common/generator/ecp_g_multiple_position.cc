/*
 * multiple_position.cpp
 *
 *  Created on: May 21, 2010
 *      Author: rtulwin
 */

#include "ecp_g_multiple_position.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

multiple_position::multiple_position(common::task::task& _ecp_task) :
	generator(_ecp_task) {

	//pose_vector = vector<ecp_mp::common::trajectory_pose::trajectory_pose>();
	//coordinate_vector = vector<vector<double> >();
}

multiple_position::~multiple_position() {
	// TODO Auto-generated destructor stub
}

void multiple_position::set_relative(void) {
	motion_type=lib::RELATIVE;
}

void multiple_position::set_absolute(void) {
	motion_type=lib::ABSOLUTE;
}

void multiple_position::set_axes_num(int axes_num) {
	this->axes_num = axes_num;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
