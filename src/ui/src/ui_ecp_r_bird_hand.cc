// -------------------------------------------------------------------------
//                            ui_ecp->cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
//
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"

#include "ui/ui_ecp_r_bird_hand.h"

#include "ecp/bird_hand/ecp_r_bird_hand.h"

// ---------------------------------------------------------------
ui_bird_hand_robot::ui_bird_hand_robot(lib::configurator &_config,
		lib::sr_ecp &_sr_ecp_msg) :
	ecp(NULL) {

	ecp = new ecp::bird_hand::robot(_config, _sr_ecp_msg);

	assert(ecp);

	// Konstruktor klasy
	ecp->ecp_command.instruction.robot_model.kinematic_model.kinematic_model_no
			= 0;
	ecp->ecp_command.instruction.get_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.get_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.set_type = ARM_DEFINITION; // ARM
	ecp->ecp_command.instruction.set_arm_type = lib::MOTOR;
	ecp->ecp_command.instruction.motion_steps = 0;
	ecp->ecp_command.instruction.value_in_step_no = 0;

	ecp->synchronised = false;
}

ui_bird_hand_robot::~ui_bird_hand_robot() {
	delete ecp;
}

// do odczytu stanu poczatkowego robota
void ui_bird_hand_robot::get_controller_state(
		lib::controller_state_t & robot_controller_initial_state_l) {
	// Zlecenie odczytu numeru modelu i korektora kinematyki
	ecp->ecp_command.instruction.instruction_type = lib::GET;
	ecp->ecp_command.instruction.get_type = CONTROLLER_STATE_DEFINITION;

	execute_motion();

	robot_controller_initial_state_l = ecp->reply_package.controller_state;
	ecp->synchronised = robot_controller_initial_state_l.is_synchronised;
}

void ui_bird_hand_robot::execute_motion(void) {

	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP


	set_ui_state_notification(UI_N_COMMUNICATION);

	// TODO: in QNX/Photon exceptions are handled at the main loop
	// in GTK exceptions triggered signals cannot be handled in main loop

	ecp->execute_motion();
}
// ---------------------------------------------------------------




