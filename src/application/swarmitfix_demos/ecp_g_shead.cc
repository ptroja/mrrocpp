/*
 * Author: yoyek
 */

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_shead.h"

namespace mrrocpp {
namespace ecp {
namespace shead {
namespace generator {

////////////////////////////////////////////////////////
//
//                  rotation_command
//
////////////////////////////////////////////////////////

//constructor with parameters: task and time to sleep [s]
rotation_command::rotation_command(task_t & _ecp_task) :
		generator_t(_ecp_task)
{
}

bool rotation_command::first_step()
{
	sr_ecp_msg.message("rotation_command: first_step");

	// parameters copying
	get_mp_ecp_command();

	the_robot->epos_joint_command_data_port.data = mp_ecp_epos_simple_command;
	the_robot->epos_joint_command_data_port.set();

	the_robot->epos_joint_reply_data_request_port.set_request();

	return true;
}

bool rotation_command::next_step()
{
	sr_ecp_msg.message("rotation_command: first_step");

	the_robot->epos_joint_reply_data_request_port.get();

	if (the_robot->epos_joint_reply_data_request_port.data.epos_controller[0].motion_in_progress) {
		the_robot->epos_joint_reply_data_request_port.set_request();

		// waits 20ms to check EPOS state
		delay(20);

		return true;
	}

	return false;

}

void rotation_command::create_ecp_mp_reply()
{
}

void rotation_command::get_mp_ecp_command()
{
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_ecp_epos_simple_command);
}

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp
