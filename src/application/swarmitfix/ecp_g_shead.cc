/*
 * generator/ecp_g_shead.cc
 *
 *Author: yoyek
 */

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

//constructor with parameters: task and time to sleep [s]
head_soldify::head_soldify(common::task::task& _ecp_task) :
	generator(_ecp_task)
{

	shead_head_soldification_data_port
			= the_robot->port_manager.get_port <lib::SHEAD_HEAD_SOLIDIFICATION> (lib::SHEAD_HEAD_SOLIDIFICATION_DATA_PORT);
	shead_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::shead_reply> (lib::SHEAD_REPLY_DATA_REQUEST_PORT);

}

bool head_soldify::first_step()
{
	// parameters copying
	get_mp_ecp_command();

	ecp_edp_shead_head_soldification_structure = mp_ecp_shead_head_soldification_structure;
	shead_head_soldification_data_port->set(ecp_edp_shead_head_soldification_structure);
	shead_reply_data_request_port->set_request();

	return true;
}

bool head_soldify::next_step()
{

	shead_reply_data_request_port->get(ecp_edp_shead_reply_structure);

	lib::SHEAD_STATE_OF_THE_HEAD motion_in_progress = ecp_edp_shead_reply_structure.head_state;

	if (motion_in_progress == lib::HEAD_STATE_INTERMEDIATE) {
		shead_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
}

void head_soldify::create_ecp_mp_reply()
{

}

void head_soldify::get_mp_ecp_command()
{
	memcpy(&mp_ecp_shead_head_soldification_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_shead_head_soldification_structure));

}

//constructor with parameters: task and time to sleep [s]
head_desoldify::head_desoldify(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	shead_head_soldification_data_port
			= the_robot->port_manager.get_port <lib::SHEAD_HEAD_SOLIDIFICATION> (lib::SHEAD_HEAD_SOLIDIFICATION_DATA_PORT);
	shead_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::shead_reply> (lib::SHEAD_REPLY_DATA_REQUEST_PORT);
}

bool head_desoldify::first_step()
{
	// parameters copying
	get_mp_ecp_command();

	ecp_edp_shead_head_soldification_structure = mp_ecp_shead_head_soldification_structure;
	shead_head_soldification_data_port->set(ecp_edp_shead_head_soldification_structure);
	shead_reply_data_request_port->set_request();

	return true;
}

bool head_desoldify::next_step()
{

	shead_reply_data_request_port->get(ecp_edp_shead_reply_structure);

	lib::SHEAD_STATE_OF_THE_HEAD motion_in_progress = ecp_edp_shead_reply_structure.head_state;

	if (motion_in_progress == lib::HEAD_STATE_INTERMEDIATE) {
		shead_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
}

void head_desoldify::create_ecp_mp_reply()
{

}

void head_desoldify::get_mp_ecp_command()
{
	memcpy(&mp_ecp_shead_head_soldification_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_shead_head_soldification_structure));
}

//constructor with parameters: task and time to sleep [s]
head_vacuum_on::head_vacuum_on(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	shead_vacuum_activation_data_port
			= the_robot->port_manager.get_port <lib::SHEAD_VACUUM_ACTIVATION> (lib::SHEAD_VACUUM_ACTIVATION_DATA_PORT);
	shead_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::shead_reply> (lib::SHEAD_REPLY_DATA_REQUEST_PORT);
}

bool head_vacuum_on::first_step()
{
	// parameters copying
	get_mp_ecp_command();

	ecp_edp_shead_vacuum_activation_structure = mp_ecp_shead_vacuum_activation_structure;
	shead_vacuum_activation_data_port->set(ecp_edp_shead_vacuum_activation_structure);
	shead_reply_data_request_port->set_request();

	return true;
}

bool head_vacuum_on::next_step()
{

	shead_reply_data_request_port->get(ecp_edp_shead_reply_structure);

	lib::SHEAD_STATE_OF_THE_VACUUM motion_in_progress = ecp_edp_shead_reply_structure.vacuum_state;

	if (motion_in_progress == lib::VACUUM_STATE_INTERMEDIATE) {
		shead_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
}

void head_vacuum_on::create_ecp_mp_reply()
{

}

void head_vacuum_on::get_mp_ecp_command()
{
	memcpy(&mp_ecp_shead_vacuum_activation_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_shead_vacuum_activation_structure));
}

//constructor with parameters: task and time to sleep [s]
head_vacuum_off::head_vacuum_off(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	shead_vacuum_activation_data_port
			= the_robot->port_manager.get_port <lib::SHEAD_VACUUM_ACTIVATION> (lib::SHEAD_VACUUM_ACTIVATION_DATA_PORT);
	shead_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::shead_reply> (lib::SHEAD_REPLY_DATA_REQUEST_PORT);
}

bool head_vacuum_off::first_step()
{
	// parameters copying
	get_mp_ecp_command();

	ecp_edp_shead_vacuum_activation_structure = mp_ecp_shead_vacuum_activation_structure;
	shead_vacuum_activation_data_port->set(ecp_edp_shead_vacuum_activation_structure);
	shead_reply_data_request_port->set_request();

	return true;
}

bool head_vacuum_off::next_step()
{
	shead_reply_data_request_port->get(ecp_edp_shead_reply_structure);

	lib::SHEAD_STATE_OF_THE_VACUUM motion_in_progress = ecp_edp_shead_reply_structure.vacuum_state;

	if (motion_in_progress == lib::VACUUM_STATE_INTERMEDIATE) {
		shead_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
}

void head_vacuum_off::create_ecp_mp_reply()
{

}

void head_vacuum_off::get_mp_ecp_command()
{
	memcpy(&mp_ecp_shead_vacuum_activation_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_shead_vacuum_activation_structure));
}

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

