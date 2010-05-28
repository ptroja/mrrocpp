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


class ui_speaker_robot;

class UiRobotSpeaker: public UiRobot {
private:

public:

	bool is_wind_speaker_play_open; // informacja czy okno odtwarzania dzwiekow jest otwarte
	ui_speaker_robot *ui_ecp_robot;

	UiRobotSpeaker();
	int reload_configuration();
	int manage_interface();

	bool pulse_reader_speaker_start_exec_pulse();
	bool pulse_reader_speaker_stop_exec_pulse();
	bool pulse_reader_speaker_trigger_exec_pulse();
};

#endif

