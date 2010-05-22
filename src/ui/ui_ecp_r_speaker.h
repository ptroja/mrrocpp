// -------------------------------------------------------------------------
//                            robot.h
// Definicje struktur danych i metod do komunikacji UI z EDP
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef _UI_ECP_R_SPEAKER_H
#define _UI_ECP_R_SPEAKER_H

#include "ui/ui.h"

#include "ecp/speaker/ecp_r_speaker.h"
// Konfigurator.
#include "lib/configurator.h"


// ---------------------------------------------------------------
class ui_speaker_robot: public ecp::speaker::robot {
// Klasa do obslugi robota irp6_on_track (sztywnego) z poziomu UI

 public:
	bool speaking_state; // stan EDP

	ui_speaker_robot (edp_state_def* edp_state, lib::configurator &_config, lib::sr_ecp &_sr_ecp_msg); // Konstruktor

	virtual void execute_motion ( void );

	// wyslanie polecenia do EDP
	bool send_command (const char* local_text, const char* local_prosody);

	// Odczyt stanu EDP
	void read_state (bool* local_state);


}; // end: class ui_speaker_robot



#endif
