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

////////////////////////////////////////////////////////
//
//                  legs_command
//
////////////////////////////////////////////////////////

//constructor with parameters: task and time to sleep [s]
legs_command::legs_command(task_t & _ecp_task) :
		generator_t(_ecp_task)
{

}

bool legs_command::first_step()
{
	// parameters copying
	get_mp_ecp_command();
	sr_ecp_msg.message("legs_command: first_step");
	the_robot->smb_festo_command_data_port.data = mp_ecp_festo_command;
	the_robot->smb_festo_command_data_port.set();
	the_robot->smb_multi_leg_reply_data_request_port.set_request();

	return true;
}

bool legs_command::next_step()
{
	the_robot->smb_multi_leg_reply_data_request_port.get();
	sr_ecp_msg.message("legs_command: next_step");
	return false;

}

void legs_command::create_ecp_mp_reply()
{

}

void legs_command::get_mp_ecp_command()
{
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_ecp_festo_command);
	//memcpy(&mp_ecp_festo_command, ecp_t.mp_command.ecp_next_state.sg_buf.data, sizeof(mp_ecp_festo_command));
}

////////////////////////////////////////////////////////
//
//                  external_epos_command
//
////////////////////////////////////////////////////////

//constructor with parameters: task and time to sleep [s]
external_epos_command::external_epos_command(task_t & _ecp_task) :
		generator_t(_ecp_task)
{
}

bool external_epos_command::first_step()
{

	// parameters copying
	get_mp_ecp_command();
	sr_ecp_msg.message("legs_command: first_step");
	the_robot->epos_external_command_data_port.data = mp_ecp_epos_simple_command;
	the_robot->epos_external_command_data_port.set();
	the_robot->epos_external_reply_data_request_port.set_request();

	return true;
}

bool external_epos_command::next_step()
{
	// waits 20ms to check epos state
	delay(20);
	the_robot->epos_external_reply_data_request_port.get();

	bool motion_in_progress = false;

	for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
		if (the_robot->epos_external_reply_data_request_port.data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		the_robot->epos_external_reply_data_request_port.set_request();
		return true;
	} else {
		return false;
	}

}

void external_epos_command::create_ecp_mp_reply()
{

}

void external_epos_command::get_mp_ecp_command()
{
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_ecp_epos_simple_command);
}

} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp

