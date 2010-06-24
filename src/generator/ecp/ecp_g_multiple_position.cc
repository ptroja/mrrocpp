/*
 * multiple_position.cc
 *
 *  Created on: May 21, 2010
 *      Author: rtulwin
 */

#include "ecp_g_multiple_position.h"

namespace mrrocpp {
namespace ecp {
namespace common {
namespace generator {

template <class Pos, class Inter, class Calc> multiple_position<Pos, Inter, Calc>::multiple_position(common::task::task& _ecp_task) :
	generator(_ecp_task) {
	// TODO Auto-generated constructor stub
}

template <class Pos, class Inter, class Calc> multiple_position<Pos, Inter, Calc>::~multiple_position() {
	// TODO Auto-generated destructor stub
}

template <class Pos, class Inter, class Calc>
void multiple_position<Pos, Inter, Calc>::set_relative(void) {
	motion_type=lib::RELATIVE;
}

template <class Pos, class Inter, class Calc>
void multiple_position<Pos, Inter, Calc>::set_absolute(void) {
	motion_type=lib::ABSOLUTE;
}

template <class Pos, class Inter, class Calc>
void multiple_position<Pos, Inter, Calc>::set_axes_num(int axes_num) {
	this->axes_num = axes_num;
}

template <class Pos, class Inter, class Calc>
void multiple_position<Pos, Inter, Calc>::reset() {
	pose_vector.clear();
	coordinate_vector.clear();
	calculated = false;
	interpolated = false;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
