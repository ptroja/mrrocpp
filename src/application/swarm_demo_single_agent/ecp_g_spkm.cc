/*
 * Author: Piotr Trojanek
 */

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace generator {

////////////////////////////////////////////////////////
//
//                  joint_epos_command
//
////////////////////////////////////////////////////////

//constructor with parameters: task and time to sleep [s]
joint_epos_command::joint_epos_command(common::task::task& _ecp_task) :
		common::generator::generator(_ecp_task)
{

	epos_joint_command_data_port =
			the_robot->port_manager.get_port <lib::epos::epos_simple_command>(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT);
	epos_joint_reply_data_request_port =
			the_robot->port_manager.get_request_port <lib::epos::epos_reply>(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT);

}

bool joint_epos_command::first_step()
{

	// parameters copying
	get_mp_ecp_command();
	sr_ecp_msg.message("legs_command: first_step");
	epos_joint_command_data_port->data = mp_ecp_epos_simple_command;
	epos_joint_command_data_port->set();
	epos_joint_reply_data_request_port->set_request();

	return true;
}

bool joint_epos_command::next_step()
{
	// waits 20ms to check epos state
	delay(20);
	epos_joint_reply_data_request_port->get();

	bool motion_in_progress = false;

	for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
		if (epos_joint_reply_data_request_port->data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		epos_joint_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}

}

void joint_epos_command::create_ecp_mp_reply()
{

}

void joint_epos_command::get_mp_ecp_command()
{
	memcpy(&mp_ecp_epos_simple_command, ecp_t.mp_command.ecp_next_state.data, sizeof(mp_ecp_epos_simple_command));
}

} // namespace generator
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

