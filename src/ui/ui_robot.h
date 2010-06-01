// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_ROBOT_H
#define __UI_ROBOT_H

#include "ui/ui.h"

class Ui;

//
//
// KLASA UiRobot
//
//


// super klasa agregujaca porozrzucane dotychczas struktury


class UiRobot {
protected:

	Ui& ui;

public:

	feb_thread* tid;
	function_execution_buffer eb;

	ecp_edp_ui_robot_def state;

	UiRobot(Ui& _ui, const std::string edp_section_name,
			const std::string ecp_section_name);
	virtual int reload_configuration()= 0;
	void create_thread();
	void abort_thread();
	bool pulse_reader_start_exec_pulse(void);
	bool pulse_reader_stop_exec_pulse(void);
	bool pulse_reader_trigger_exec_pulse(void);
	virtual int close_all_windows();
	int EDP_slay_int();
	virtual int delete_ui_ecp_robot() = 0;

};

#endif

