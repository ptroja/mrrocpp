// -------------------------------------------------------------------------
//                            ui_ecp.cc
// Metody sluzace do komunikacji UI z EDP - zlecenia dla driver'a
// 
// Ostatnio modyfikowany: 2005
// -------------------------------------------------------------------------

/* Standard headers */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <iostream>

#include "common/typedefs.h"
#include "common/impconst.h"
#include "common/com_buf.h"

#include "lib/srlib.h"
#include "ui/ui_ecp_r_speaker.h"

#include <math.h>
#include "lib/mathtr.h"

/* Local headers */
#include "proto.h"

// ---------------------------------------------------------------
ui_speaker_robot::ui_speaker_robot (edp_state_def* _edp_state, configurator &_config, sr_ecp* sr_ecp_msg) 
	: ecp_speaker_robot (_config, sr_ecp_msg) { 

  // Konstruktor klasy
  EDP_command_and_reply_buffer.sr_ecp_msg = sr_ecp_msg;

  synchronised = true;

};// end: ui_conveyor_robot::ui_conveyor_robot ()
// ---------------------------------------------------------------



void ui_speaker_robot::execute_motion (void) { 
// Zlecenie wykonania ruchu przez robota jest to polecenie dla EDP
	set_ui_state_notification(UI_N_COMMUNICATION);

	ecp_robot::execute_motion();

}; // end: conveyor_robot::execute_motion (void)
// ---------------------------------------------------------------


bool ui_speaker_robot::send_command (const char* local_text, const char* local_prosody) { 

	EDP_command_and_reply_buffer.instruction.instruction_type = SET;

	if ((local_text)&&(local_prosody)) 
	{
		strncpy(EDP_command_and_reply_buffer.instruction.arm.text_def.text, local_text, MAX_TEXT);
		strncpy(EDP_command_and_reply_buffer.instruction.arm.text_def.prosody, local_prosody, MAX_PROSODY );
	}
	
	execute_motion();

	return true;
}
	
bool ui_speaker_robot::read_state (int* local_state) { 

	EDP_command_and_reply_buffer.instruction.instruction_type = GET;

	execute_motion();
	    
	    if (local_state) {
		    *local_state=EDP_command_and_reply_buffer.reply_package.arm.text_def.speaking;
		}
	   // printf("UI SPEAKING: %d\n", EDP_command_and_reply_buffer.reply_package.arm.text_def.speaking);
	    
	return true;
	
}
