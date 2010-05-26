// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_CLASS_H
#define __UI_CLASS_H

#include "ui/ui.h"
#include "ui/ui_r_bird_hand.h"
#include "lib/configurator.h"

//
//
// KLASA ui
//
//


// super klasa agregujaca porozrzucane struktury


class Ui {
private:

public:
	boost::mutex process_creation_mtx;
	lib::configurator* config;
	lib::sr_ecp* all_ecp_msg; // Wskaznik na obiekt do komunikacji z SR z fukcja ECP dla wszystkich robotow
	lib::sr_ui* ui_msg; // Wskaznik na obiekt do komunikacji z SR

	mp_state_def mp;
	// bool is_any_edp_active;
	bool is_mp_and_ecps_active;

	UI_ALL_EDPS_STATE all_edps;

	UiRobotBirdHand bird_hand;

	Ui();
	int manage_interface(void);

};

#endif

