/*
 * Author: Piotr Trojanek
 */

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_smb.h"

namespace mrrocpp {
namespace ecp {
namespace smb {
namespace generator {

//
//
//
// quickstop
//
//
//

quickstop::quickstop(task_t & _ecp_task) :
		generator_t(_ecp_task)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

bool quickstop::first_step()
{
	//the_robot->epos_brake_command_data_port.data = true;
	the_robot->epos_brake_command_data_port.set();

	return true;
}

bool quickstop::next_step()
{
	return true;
}

////////////////////////////////////////////////////////
//
//                  legs_command
//
////////////////////////////////////////////////////////

stand_up::stand_up(task_t & _ecp_task,
		const lib::smb::festo_command_td & cmd) :
		generator_t(_ecp_task)
{
	// Keep internal copy of a command
	festo_command = cmd;
}

bool stand_up::first_step()
{
	sr_ecp_msg.message("legs_command: first_step");

	the_robot->smb_festo_command_data_port.data = festo_command;
	the_robot->smb_festo_command_data_port.set();

	the_robot->smb_multi_leg_reply_data_request_port.set_request();

	return true;
}

bool stand_up::next_step()
{
	the_robot->smb_multi_leg_reply_data_request_port.get();
	sr_ecp_msg.message("legs_command: next_step");
	return false;
}

////////////////////////////////////////////////////////
//
//                  external_epos_command
//
////////////////////////////////////////////////////////

rotate::rotate(task_t & _ecp_task,
		const lib::smb::motor_command & cmd) :
		generator_t(_ecp_task),
		wakeup(20)
{
	// Keep internal copy of a command
	simple_command = cmd;
}

bool rotate::first_step()
{
	sr_ecp_msg.message("legs_command: first_step");
	the_robot->epos_external_command_data_port.data = simple_command;
	the_robot->epos_external_command_data_port.set();
	the_robot->epos_external_reply_data_request_port.set_request();

	return true;
}

bool rotate::next_step()
{
	the_robot->epos_external_reply_data_request_port.get();

	bool motion_in_progress = false;

	for (int i = 0; i < lib::smb::NUM_OF_SERVOS; i++) {
		if (the_robot->epos_external_reply_data_request_port.data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		// Wait so not to query EDP too often
		wakeup.sleep();

		// Ask about current status
		the_robot->epos_external_reply_data_request_port.set_request();
		return true;
	}

	return false;
}

} // namespace generator
} // namespace smb
} // namespace ecp
} // namespace mrrocpp
