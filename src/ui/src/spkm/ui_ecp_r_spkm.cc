// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <iostream>

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstring>
#include <cassert>
#include <fcntl.h>
#include <cerrno>
#include <cmath>

#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"

#include "base/lib/sr/srlib.h"

#include "ui/src/spkm/ui_ecp_r_spkm.h"

namespace mrrocpp {
namespace ui {
namespace spkm {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg) :
	the_robot(NULL)
{

	the_robot = new ecp::spkm::robot(_config, _sr_ecp_msg);

	epos_cubic_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_cubic_command> (lib::epos::EPOS_CUBIC_COMMAND_DATA_PORT);

	epos_trapezoidal_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_trapezoidal_command> (lib::epos::EPOS_TRAPEZOIDAL_COMMAND_DATA_PORT);

	epos_operational_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_operational_command> (lib::epos::EPOS_OPERATIONAL_COMMAND_DATA_PORT);

	epos_brake_command_data_port = the_robot->port_manager.get_port <bool> (lib::epos::EPOS_BRAKE_COMMAND_DATA_PORT);

	epos_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::epos::epos_reply> (lib::epos::EPOS_REPLY_DATA_REQUEST_PORT);

	assert(the_robot);

}

EcpRobot::~EcpRobot()
{
	delete the_robot;
}

// do odczytu stanu poczatkowego robota
void EcpRobot::get_controller_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki


	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = CONTROLLER_STATE_DEFINITION;

	the_robot->execute_motion();

	robot_controller_initial_state_l = the_robot->reply_package.controller_state;
	the_robot->synchronised = robot_controller_initial_state_l.is_synchronised;
}

void EcpRobot::execute_motion(void)
{

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP

	set_ui_state_notification(UI_N_COMMUNICATION);

	the_robot->create_command();

	the_robot->execute_motion();

	the_robot->get_reply();
}
// ---------------------------------------------------------------

}
} //namespace ui
} //namespace mrrocpp
