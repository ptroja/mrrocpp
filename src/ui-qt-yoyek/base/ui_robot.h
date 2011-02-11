// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_ROBOT_H
#define __UI_ROBOT_H

#include "ui.h"

class QDockWidget;

namespace mrrocpp {
namespace ui {
namespace common {

class Interface;

typedef std::map <std::string, QDockWidget*> WndBase_t;
typedef WndBase_t::value_type WndBase_pair_t;

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

	/**
	 * @brief Unique robot name
	 */
	const lib::robot_name_t robot_name; // by Y - nazwa robota (track, postument etc.)
	int number_of_servos;
	std::string activation_string;

	common::WndBase_t wndbase_m;

			UiRobot(Interface& _interface, const std::string & edp_section_name, const std::string & ecp_section_name, lib::robot_name_t _robot_name, int _number_of_servos, const std::string & _activation_string);

	void create_thread();
	void abort_thread();
	void pulse_reader_execute(int code, int value);

	bool pulse_reader_start_exec_pulse(void);
	bool pulse_reader_stop_exec_pulse(void);
	bool pulse_reader_trigger_exec_pulse(void);

	void pulse_ecp(void);
	void close_all_windows();
	void EDP_slay_int();
	void connect_to_reader(void);
	void connect_to_ecp_pulse_chanell(void); //TODO: channel, nie chanell
	void pulse_ecp_execute(int code, int value);
	virtual void delete_ui_ecp_robot() = 0;
	virtual int manage_interface() = 0;

	virtual int synchronise() = 0;
	virtual void edp_create() = 0;
	virtual int edp_create_int() = 0;

	bool check_synchronised_and_loaded();
	bool deactivate_ecp_trigger();
	int reload_configuration();

	virtual int move_to_synchro_position();
	virtual int move_to_front_position();
	virtual int move_to_preset_position(int variant);

};

}
}
}
#endif

