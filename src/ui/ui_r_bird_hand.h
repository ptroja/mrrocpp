// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_BIRD_HAND_H
#define __UI_R_BIRD_HAND_H

#include "ui/ui.h"
#include "ui/ui_r_bird_hand.h"

//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobotBirdHand {
private:

public:

	bool is_wnd_bird_hand_command_and_status_open;
	bool is_wnd_bird_hand_configuration_open;

	ecp_edp_ui_robot_def state;

	UiRobotBirdHand();
	int reload_configuration();
};

#endif

