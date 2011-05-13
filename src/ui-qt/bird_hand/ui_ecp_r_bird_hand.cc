#include "base/lib/typedefs.h"
#include "base/lib/impconst.h"
#include "base/lib/com_buf.h"
#include "../base/interface.h"

#include "base/lib/sr/srlib.h"

#include "ui_ecp_r_bird_hand.h"

namespace mrrocpp {
namespace ui {
namespace bird_hand {

// ---------------------------------------------------------------
EcpRobot::EcpRobot(common::UiRobot& _ui_robot) :
	EcpRobotDataPort(_ui_robot)
{

	the_robot = (boost::shared_ptr <robot_t>) new ecp::bird_hand::robot(*(ui_robot.interface.config), *(ui_robot.msg));

	bird_hand_command_data_port
			= the_robot->port_manager.get_port <lib::bird_hand::command> (lib::bird_hand::COMMAND_DATA_PORT);

	bird_hand_configuration_command_data_port
			= the_robot->port_manager.get_port <lib::bird_hand::configuration> (lib::bird_hand::CONFIGURATION_DATA_PORT);

	bird_hand_status_reply_data_request_port
			= the_robot->port_manager.get_request_port <lib::bird_hand::status> (lib::bird_hand::STATUS_DATA_REQUEST_PORT);

	bird_hand_configuration_reply_data_request_port = the_robot->port_manager.get_request_port <
			lib::bird_hand::configuration> (lib::bird_hand::CONFIGURATION_DATA_REQUEST_PORT);
}

}
} //namespace ui
} //namespace mrrocpp
