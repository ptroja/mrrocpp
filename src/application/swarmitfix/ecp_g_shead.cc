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

	shead_head_soldification_data_port = the_robot->port_manager.get_port<
			lib::SHEAD_HEAD_SOLIDIFICATION> (
			SHEAD_HEAD_SOLIDIFICATION_DATA_PORT);
	shead_reply_data_request_port = the_robot->port_manager.get_request_port<
			lib::shead_reply> (SHEAD_REPLY_DATA_REQUEST_PORT);

}

bool head_soldify::first_step() {

	return true;
}

bool head_soldify::next_step() {
	return true;
}

void head_soldify::create_ecp_mp_reply() {

}

void head_soldify::get_mp_ecp_command() {

}

//constructor with parameters: task and time to sleep [s]
head_desoldify::head_desoldify(common::task::task& _ecp_task) :
	generator(_ecp_task) {
	shead_head_soldification_data_port = the_robot->port_manager.get_port<
			lib::SHEAD_HEAD_SOLIDIFICATION> (
			SHEAD_HEAD_SOLIDIFICATION_DATA_PORT);
	shead_reply_data_request_port = the_robot->port_manager.get_request_port<
			lib::shead_reply> (SHEAD_REPLY_DATA_REQUEST_PORT);
}

bool head_desoldify::first_step() {

	return true;
}

bool head_desoldify::next_step() {
	return true;
}

void head_desoldify::create_ecp_mp_reply() {

}

void head_desoldify::get_mp_ecp_command() {

}

//constructor with parameters: task and time to sleep [s]
head_vacuum_on::head_vacuum_on(common::task::task& _ecp_task) :
	generator(_ecp_task) {
	shead_vacuum_activation_data_port = the_robot->port_manager.get_port<
			lib::SHEAD_VACUUM_ACTIVATION> (SHEAD_VACUUM_ACTIVATION_DATA_PORT);
	shead_reply_data_request_port = the_robot->port_manager.get_request_port<
			lib::shead_reply> (SHEAD_REPLY_DATA_REQUEST_PORT);
}

bool head_vacuum_on::first_step() {
	return true;
}

bool head_vacuum_on::next_step() {
	return true;
}

void head_vacuum_on::create_ecp_mp_reply() {

}

void head_vacuum_on::get_mp_ecp_command() {

}

//constructor with parameters: task and time to sleep [s]
head_vacuum_off::head_vacuum_off(common::task::task& _ecp_task) :
	generator(_ecp_task) {
	shead_vacuum_activation_data_port = the_robot->port_manager.get_port<
			lib::SHEAD_VACUUM_ACTIVATION> (SHEAD_VACUUM_ACTIVATION_DATA_PORT);
	shead_reply_data_request_port = the_robot->port_manager.get_request_port<
			lib::shead_reply> (SHEAD_REPLY_DATA_REQUEST_PORT);
}

bool head_vacuum_off::first_step() {

	return true;
}

bool head_vacuum_off::next_step() {
	return true;
}

void head_vacuum_off::create_ecp_mp_reply() {

}

void head_vacuum_off::get_mp_ecp_command() {

}

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

