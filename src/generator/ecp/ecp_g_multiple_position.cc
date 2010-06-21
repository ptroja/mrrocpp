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

template <class T> multiple_position<T>::multiple_position(common::task::task& _ecp_task) :
	generator(_ecp_task) {
	// TODO Auto-generated constructor stub
}

template <class T> multiple_position<T>::~multiple_position() {
	// TODO Auto-generated destructor stub
}

template <class T>
void multiple_position<T>::set_relative(void) {
	motion_type=lib::RELATIVE;
}

template <class T>
void multiple_position<T>::set_absolute(void) {
	motion_type=lib::ABSOLUTE;
}

template <class T>
void multiple_position<T>::set_axes_num(int axes_num) {
	this->axes_num = axes_num;
}

} // namespace generator
} // namespace common
} // namespace ecp
} // namespace mrrocpp
