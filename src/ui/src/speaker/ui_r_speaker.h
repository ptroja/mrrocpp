// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_R_SPEAKER_H
#define __UI_R_SPEAKER_H

#include "ui/ui.h"
#include "ui/ui_robot.h"

//
//
// KLASA UiRobotSpeaker
//
//


// super klasa agregujaca porozrzucane struktury

class Ui;
class ui_speaker_robot;

class UiRobotSpeaker: public UiRobot {
private:

public:

	bool is_wind_speaker_play_open; // informacja czy okno odtwarzania dzwiekow jest otwarte
	ui_speaker_robot *ui_ecp_robot;

	UiRobotSpeaker(Ui& _ui);
	int reload_configuration();
	int manage_interface();

	int close_all_windows();
	int delete_ui_ecp_robot();

};

#endif

