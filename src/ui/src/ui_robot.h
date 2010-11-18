// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_ROBOT_H
#define __UI_ROBOT_H

#include "ui/src/ui.h"

namespace mrrocpp {
namespace ui {
namespace common {

class Interface;

//
//
// KLASA UiRobot
//
//


// super klasa agregujaca porozrzucane dotychczas struktury


class UiRobot
{
protected:

	Interface& interface;

public:
	feb_thread* tid;
	function_execution_buffer eb;

	ecp_edp_ui_robot_def state;

	UiRobot(Interface& _interface, const std::string & edp_section_name, const std::string & ecp_section_name);
	virtual int reload_configuration()= 0;
	void create_thread();
	void abort_thread();
	void pulse_reader_execute(int code, int value);

	bool pulse_reader_start_exec_pulse(void);
	bool pulse_reader_stop_exec_pulse(void);
	bool pulse_reader_trigger_exec_pulse(void);

	void pulse_ecp(void);
	virtual void close_all_windows();
	void EDP_slay_int();
	void connect_to_reader(void);
	void connect_to_ecp_pulse_chanell(void);
	void pulse_ecp_execute(int code, int value);
	virtual void delete_ui_ecp_robot() = 0;

	bool check_synchronised_or_inactive();
	bool check_synchronised_and_loaded();
	bool check_loaded_or_inactive();
	bool check_loaded();
	bool deactivate_ecp_trigger();
};

}
}
}
#endif

