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

#include "base/lib/srlib.h"

#include "ui/src/bird_hand/ui_ecp_r_bird_hand.h"

// ---------------------------------------------------------------
ui_bird_hand_robot::ui_bird_hand_robot(lib::configurator &_config,
		lib::sr_ecp &_sr_ecp_msg) :
	the_robot(NULL) {

	the_robot = new ecp::bird_hand::robot(_config, _sr_ecp_msg);

	bird_hand_command_data_port = the_robot->port_manager.get_port<
			lib::bird_hand::command> (BIRD_HAND_COMMAND_DATA_PORT);

	bird_hand_configuration_command_data_port
			= the_robot->port_manager.get_port<lib::bird_hand::configuration> (
					BIRD_HAND_CONFIGURATION_DATA_PORT);

	bird_hand_status_reply_data_request_port
			= the_robot->port_manager.get_request_port<lib::bird_hand::status> (
					BIRD_HAND_STATUS_DATA_REQUEST_PORT);

	bird_hand_configuration_reply_data_request_port
			= the_robot->port_manager.get_request_port<
					lib::bird_hand::configuration> (
					BIRD_HAND_CONFIGURATION_DATA_REQUEST_PORT);

	assert(the_robot);

}

ui_bird_hand_robot::~ui_bird_hand_robot() {
	delete the_robot;
}

// do odczytu stanu poczatkowego robota
void ui_bird_hand_robot::get_controller_state(
		lib::controller_state_t & robot_controller_initial_state_l) {
	// Zlecenie odczytu numeru modelu i korektora kinematyki


	the_robot->ecp_command.instruction.instruction_type = lib::GET;
	the_robot->ecp_command.instruction.get_type = CONTROLLER_STATE_DEFINITION;

	the_robot->execute_motion();

	robot_controller_initial_state_l
			= the_robot->reply_package.controller_state;
	the_robot->synchronised = robot_controller_initial_state_l.is_synchronised;
}

void ui_bird_hand_robot::execute_motion(void) {

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP

	set_ui_state_notification(UI_N_COMMUNICATION);

	the_robot->create_command();

	the_robot->execute_motion();

	the_robot->get_reply();
}
// ---------------------------------------------------------------


