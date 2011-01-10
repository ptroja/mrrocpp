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

#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"

namespace mrrocpp {
namespace ui {
namespace bird_hand {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::Interface& _interface) :
	common::EcpRobotDataPort(_interface)
{

	the_robot = new ecp::bird_hand::robot(*(_interface.config), *(_interface.all_ecp_msg));

	bird_hand_command_data_port
			= the_robot->port_manager.get_port <lib::bird_hand::command> (lib::bird_hand::COMMAND_DATA_PORT);

	bird_hand_configuration_command_data_port
			= the_robot->port_manager.get_port <lib::bird_hand::configuration> (lib::bird_hand::CONFIGURATION_DATA_PORT);

	bird_hand_status_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::bird_hand::status> (lib::bird_hand::STATUS_DATA_REQUEST_PORT);

	bird_hand_configuration_reply_data_request_port = the_robot->port_manager.get_request_port <
			lib::bird_hand::configuration> (lib::bird_hand::CONFIGURATION_DATA_REQUEST_PORT);

	assert(the_robot);

}

}
} //namespace ui
} //namespace mrrocpp
