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
	common::EcpRobotDataPort()
{

	the_robot = new ecp::spkm::robot(_config, _sr_ecp_msg);

	epos_motor_command_data_port
			= the_robot->port_manager.get_port <lib::epos::epos_motor_command> (lib::epos::EPOS_MOTOR_COMMAND_DATA_PORT);

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

// ---------------------------------------------------------------
void EcpRobot::move_motors(const double final_position[])
{
	for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
		epos_motor_command_data_port->data.desired_position[i] = final_position[i];
	}
	//	std::cout << "UI final_position[4]" << final_position[4] << std::endl;
	epos_motor_command_data_port->set();
	execute_motion();

}
// ---------------------------------------------------------------

}
} //namespace ui
} //namespace mrrocpp
