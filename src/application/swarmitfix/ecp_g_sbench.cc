/*
 * Author: Piotr Trojanek
 */

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_sbench.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {
namespace generator {

//
//
//
// pin_config
//
//
//

pin_config::pin_config(task_t & _ecp_task, const lib::sbench::voltage_buffer & _pins) :
		generator_t(_ecp_task),
		pin_configuration(_pins)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

bool pin_config::first_step()
{
	sr_ecp_msg.message("pin_config: first_step");

	// Forward coordinator's command
	the_robot->sbench_command_voltage_data_port.data = pin_configuration;
	the_robot->sbench_command_voltage_data_port.set();

	// Request status report
	the_robot->sbench_reply_data_request_port.set_request();

	return true;
}

bool pin_config::next_step()
{
	// End
	return false;
}

} // namespace generator
} // namespace sbench
} // namespace ecp
} // namespace mrrocpp
