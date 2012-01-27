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

power_supply::power_supply(task_t & _ecp_task, const lib::sbench::power_supply_state & _pins) :
		generator_t(_ecp_task),
		pin_configuration(_pins)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

bool power_supply::first_step()
{
	sr_ecp_msg.message("pin_config: first_step");

	// Forward coordinator's command
	the_robot->power_supply_data_port.data = pin_configuration;
	the_robot->power_supply_data_port.set();

	// Request status report
	the_robot->data_request_port.set_request();

	return true;
}

bool power_supply::next_step()
{
	// End
	return false;
}

//
//
//
// cleaning
//
//
//

cleaning::cleaning(task_t & _ecp_task, const lib::sbench::cleaning_state & _pins) :
		generator_t(_ecp_task),
		pin_configuration(_pins)
{
	//	if (the_robot) the_robot->communicate_with_edp = false; //do not communicate with edp
}

bool cleaning::first_step()
{
	sr_ecp_msg.message("pin_config: first_step");

	// Forward coordinator's command
	the_robot->cleaning_state_data_port.data = pin_configuration;
	the_robot->cleaning_state_data_port.set();

	// Request status report
	the_robot->data_request_port.set_request();

	return true;
}

bool cleaning::next_step()
{
	// End
	return false;
}

} // namespace generator
} // namespace sbench
} // namespace ecp
} // namespace mrrocpp
