// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_BIRD_HAND_H
#define __UI_R_BIRD_HAND_H

#include "ui/ui.h"
#include "ui/ui_robot.h"

//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury

class ui_bird_hand_robot;

class UiRobotBirdHand: public UiRobot {
private:

public:

	ui_bird_hand_robot *ui_ecp_robot;

	bool is_wnd_bird_hand_command_and_status_open;
	bool is_wnd_bird_hand_configuration_open;

	UiRobotBirdHand();
	int reload_configuration();
	int manage_interface();
	int close_all_windows();

	int get_index_f_0_command();
	int set_index_f_0_status();

};

#endif

