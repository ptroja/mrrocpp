// -------------------------------------------------------------------------
//                            ui_ecp.cc
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
#include <math.h>
#include <fcntl.h>
#include <errno.h>

#include "lib/typedefs.h"
#include "lib/impconst.h"
#include "lib/com_buf.h"

#include "lib/srlib.h"
#include "lib/mrmath/mrmath.h"

#include "ui/src/speaker/ui_ecp_r_speaker.h"

ui_speaker_robot::ui_speaker_robot(edp_state_def* _edp_state,
		lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg) :
	robot(_config, sr_ecp_msg) {
	// This has to be set in constructor since it is a field in a base class
	synchronised = true;
}

void ui_speaker_robot::execute_motion(void) {
	// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
	set_ui_state_notification(UI_N_COMMUNICATION);

	ecp_robot::execute_motion();
}

bool ui_speaker_robot::send_command(const char* local_text,
		const char* local_prosody) {
	ecp_command.instruction.instruction_type = lib::SET;

	if ((local_text) && (local_prosody)) {
		strncpy(ecp_command.instruction.arm.text_def.text, local_text, MAX_TEXT);
		strncpy(ecp_command.instruction.arm.text_def.prosody, local_prosody,
				MAX_PROSODY );
	}

	execute_motion();

	return true;
}

void ui_speaker_robot::read_state(bool* local_state) {
	ecp_command.instruction.instruction_type = lib::GET;

	execute_motion();

	if (local_state) {
		*local_state = reply_package.arm.text_def.speaking;
	}
	// printf("UI SPEAKING: %d\n", reply_package.arm.text_def.speaking);
}



