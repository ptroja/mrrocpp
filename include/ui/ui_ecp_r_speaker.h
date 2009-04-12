// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
// 
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_SPEAKER_H
#define _UI_ECP_R_SPEAKER_H

#include "ecp/speaker/ecp_local.h"

#include "ui/ui.h"
// Konfigurator.
#include "lib/configurator.h"


// ---------------------------------------------------------------
class ui_speaker_robot: public ecp::speaker::ecp_speaker_robot {
// Klasa do obslugi robota irp6_on_track (sztywnego) z poziomu UI

 public:
	int speaking_state; // stan EDP
	
	ui_speaker_robot (edp_state_def* edp_state, configurator &_config, sr_ecp* _sr_ecp_msg); // Konstruktor
	
	virtual void execute_motion ( void );
	
	// wyslanie polecenia do EDP
	bool send_command (const char* local_text, const char* local_prosody);
	
	// Odczyt stanu EDP 
	bool read_state (int* local_state);
	

}; // end: class ui_speaker_robot



#endif
