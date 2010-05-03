/*
 * generator/ecp_g_sleep.cc
 *
 *Author: Tomasz Bem
 */

#include "ecp_g_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

//constructor with parameters: task and time to sleep [s]
head_soldify::head_soldify(common::task::task& _ecp_task) :
	generator(_ecp_task) {

}

bool head_soldify::first_step() {

	return true;
}

bool head_soldify::next_step() {
	return true;
}

//constructor with parameters: task and time to sleep [s]
head_desoldify::head_desoldify(common::task::task& _ecp_task) :
	generator(_ecp_task) {

}

bool head_desoldify::first_step() {

	return true;
}

bool head_desoldify::next_step() {
	return true;
}

//constructor with parameters: task and time to sleep [s]
head_vacuum_on::head_vacuum_on(common::task::task& _ecp_task) :
	generator(_ecp_task) {

}

bool head_vacuum_on::first_step() {
	return true;
}

bool head_vacuum_on::next_step() {
	return true;
}

//constructor with parameters: task and time to sleep [s]
head_vacuum_off::head_vacuum_off(common::task::task& _ecp_task) :
	generator(_ecp_task) {

}

bool head_vacuum_off::first_step() {

	return true;
}

bool head_vacuum_off::next_step() {
	return true;
}

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

