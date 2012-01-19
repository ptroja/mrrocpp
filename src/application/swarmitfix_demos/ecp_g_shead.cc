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
//                  joint_epos_command
//
////////////////////////////////////////////////////////

//constructor with parameters: task and time to sleep [s]
sbench_transparent_generator::sbench_transparent_generator(task_t & _ecp_task) :
		generator_t(_ecp_task)
{

	sbench_data_port =
			the_robot->port_manager.get_port <lib::epos::epos_simple_command>(lib::epos::EPOS_JOINT_COMMAND_DATA_PORT);
	sbench_reply_data_request_port =
			the_robot->port_manager.get_request_port <lib::epos::epos_reply>(lib::epos::EPOS_JOINT_REPLY_DATA_REQUEST_PORT);

}

bool sbench_transparent_generator::first_step()
{

	// parameters copying
	get_mp_ecp_command();
	sr_ecp_msg.message("legs_command: first_step");
	sbench_data_port->data = mp_ecp_epos_simple_command;
	sbench_data_port->set();
	sbench_reply_data_request_port->set_request();

	return true;
}

bool sbench_transparent_generator::next_step()
{
	// waits 20ms to check epos state
	delay(20);
	sbench_reply_data_request_port->get();

	bool motion_in_progress = false;

	for (int i = 0; i < lib::shead::NUM_OF_SERVOS; i++) {
		if (sbench_reply_data_request_port->data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		sbench_reply_data_request_port->set_request();
		return true;
	} else {
		return false;
	}

}

void sbench_transparent_generator::create_ecp_mp_reply()
{

}

void sbench_transparent_generator::get_mp_ecp_command()
{
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_ecp_epos_simple_command);
	//memcpy(&mp_ecp_epos_simple_command, ecp_t.mp_command.ecp_next_state.sg_buf.data, sizeof(mp_ecp_epos_simple_command));
}

} // namespace generator
} // namespace shead
} // namespace ecp
} // namespace mrrocpp

