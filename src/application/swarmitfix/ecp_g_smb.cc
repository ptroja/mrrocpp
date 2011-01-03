/*
 * generator/ecp_g_smb.cc
 *
 *Author: yoyek
 */

#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

//constructor with parameters: task and time to sleep [s]
pin_lock::pin_lock(common::task::task& _ecp_task) :
	generator(_ecp_task)
{

	smb_multi_pin_locking_data_port
			= the_robot->port_manager.get_port <lib::smb::multi_pin_locking_td> (lib::smb::MULTI_PIN_LOCKING_DATA_PORT);
	smb_multi_leg_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::smb::multi_leg_reply_td> (lib::smb::MULTI_LEG_REPLY_DATA_REQUEST_PORT);

}

bool pin_lock::first_step()
{
	// parameters copying
	get_mp_ecp_command();

	smb_multi_pin_locking_data_port->data = mp_ecp_smb_multi_pin_locking_structure;
	smb_multi_pin_locking_data_port->set();
	smb_multi_leg_reply_data_request_port->set_request();

	return true;
}

bool pin_lock::next_step()
{
	smb_multi_leg_reply_data_request_port->get();

	bool motion_in_progress = false;

	for (int i = 0; i < 3; i++) {
		if (smb_multi_leg_reply_data_request_port->data.leg[i].locking_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		smb_multi_leg_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
}

void pin_lock::create_ecp_mp_reply()
{

}

void pin_lock::get_mp_ecp_command()
{
	memcpy(&mp_ecp_smb_multi_pin_locking_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_smb_multi_pin_locking_structure));
}

//constructor with parameters: task and time to sleep [s]
pin_unlock::pin_unlock(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	smb_multi_pin_locking_data_port
			= the_robot->port_manager.get_port <lib::smb::multi_pin_locking_td> (lib::smb::MULTI_PIN_LOCKING_DATA_PORT);
	smb_multi_leg_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::smb::multi_leg_reply_td> (lib::smb::MULTI_LEG_REPLY_DATA_REQUEST_PORT);
}

bool pin_unlock::first_step()
{
	// parameters copying
	get_mp_ecp_command();
	smb_multi_pin_locking_data_port->data = mp_ecp_smb_multi_pin_locking_structure;
	smb_multi_pin_locking_data_port->set();
	smb_multi_leg_reply_data_request_port->set_request();
	return true;
}

bool pin_unlock::next_step()
{

	smb_multi_leg_reply_data_request_port->get();

	bool motion_in_progress = false;

	for (int i = 0; i < 3; i++) {
		if (smb_multi_leg_reply_data_request_port->data.leg[i].locking_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		smb_multi_leg_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
}

void pin_unlock::create_ecp_mp_reply()
{

}

void pin_unlock::get_mp_ecp_command()
{
	memcpy(&mp_ecp_smb_multi_pin_locking_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_smb_multi_pin_locking_structure));
}

//constructor with parameters: task and time to sleep [s]
pin_rise::pin_rise(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	smb_multi_pin_insertion_data_port
			= the_robot->port_manager.get_port <lib::smb::multi_pin_insertion_td> (lib::smb::MULTI_PIN_INSERTION_DATA_PORT);
	smb_multi_leg_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::smb::multi_leg_reply_td> (lib::smb::MULTI_LEG_REPLY_DATA_REQUEST_PORT);

}

bool pin_rise::first_step()
{
	// parameters copying
	get_mp_ecp_command();

	smb_multi_pin_insertion_data_port->data = mp_ecp_smb_multi_pin_insertion_structure;
	smb_multi_pin_insertion_data_port->set();
	smb_multi_leg_reply_data_request_port->set_request();

	return true;
}

bool pin_rise::next_step()
{

	smb_multi_leg_reply_data_request_port->get();

	bool motion_in_progress = false;

	for (int i = 0; i < 3; i++) {
		if (smb_multi_leg_reply_data_request_port->data.leg[i].insertion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		smb_multi_leg_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
}

void pin_rise::create_ecp_mp_reply()
{

}

void pin_rise::get_mp_ecp_command()
{
	memcpy(&mp_ecp_smb_multi_pin_insertion_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_smb_multi_pin_insertion_structure));
}

//constructor with parameters: task and time to sleep [s]
pin_lower::pin_lower(common::task::task& _ecp_task) :
	generator(_ecp_task)
{
	smb_multi_pin_insertion_data_port
			= the_robot->port_manager.get_port <lib::smb::multi_pin_insertion_td> (lib::smb::MULTI_PIN_INSERTION_DATA_PORT);
	smb_multi_leg_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::smb::multi_leg_reply_td> (lib::smb::MULTI_LEG_REPLY_DATA_REQUEST_PORT);

}

bool pin_lower::first_step()
{
	// parameters copying
	get_mp_ecp_command();

	smb_multi_pin_insertion_data_port->data = mp_ecp_smb_multi_pin_insertion_structure;
	smb_multi_pin_insertion_data_port->set();
	smb_multi_leg_reply_data_request_port->set_request();

	return true;
}

bool pin_lower::next_step()
{

	smb_multi_leg_reply_data_request_port->get();

	bool motion_in_progress = false;

	for (int i = 0; i < 3; i++) {
		if (smb_multi_leg_reply_data_request_port->data.leg[i].insertion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		smb_multi_leg_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}
}

void pin_lower::create_ecp_mp_reply()
{

}

void pin_lower::get_mp_ecp_command()
{
	memcpy(&mp_ecp_smb_multi_pin_insertion_structure, ecp_t.mp_command.ecp_next_state.mp_2_ecp_next_state_string, sizeof(mp_ecp_smb_multi_pin_insertion_structure));
}

} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

