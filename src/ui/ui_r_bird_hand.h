// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_BIRD_HAND_H
#define __UI_R_BIRD_HAND_H

#include "ui/ui.h"

class ui_bird_hand_robot;

//
//
// KLASA UiRobotBirdHand
//
//


// super klasa agregujaca porozrzucane struktury


class UiRobotBirdHand {
private:

public:

	feb_thread* tid;
	function_execution_buffer eb;

	ui_bird_hand_robot *ui_ecp_robot;

	bool is_wnd_bird_hand_command_and_status_open;
	bool is_wnd_bird_hand_configuration_open;

	ecp_edp_ui_robot_def state;

	UiRobotBirdHand();
	int reload_configuration();
	void create_thread();
	void abort_thread();
};

#endif

