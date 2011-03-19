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
#include "ui/src/ui_class.h"

#include "base/lib/sr/srlib.h"

#include "ui/src/ui_ecp_r_data_port.h"

namespace mrrocpp {
namespace ui {
namespace common {

// ---------------------------------------------------------------
EcpRobotDataPort::EcpRobotDataPort(Interface& _interface) :
	interface(_interface), the_robot(NULL)
{

}

EcpRobotDataPort::~EcpRobotDataPort()
{
	delete the_robot;
}

// do odczytu stanu poczatkowego robota
void EcpRobotDataPort::get_controller_state(lib::controller_state_t & robot_controller_initial_state_l)
{
	// Zlecenie odczytu numeru modelu i korektora kinematyki


	the_robot->ecp_command.instruction_type = lib::GET;
	the_robot->ecp_command.get_type = CONTROLLER_STATE_DEFINITION;

	the_robot->execute_motion();

	robot_controller_initial_state_l = the_robot->reply_package.controller_state;
	the_robot->synchronised = robot_controller_initial_state_l.is_synchronised;
}

void EcpRobotDataPort::execute_motion(void)
{

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP

	interface.set_ui_state_notification(UI_N_COMMUNICATION);

	the_robot->create_command();

	the_robot->execute_motion();

	the_robot->get_reply();
}
// ---------------------------------------------------------------

}
} //namespace ui
} //namespace mrrocpp
