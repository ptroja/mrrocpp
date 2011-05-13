// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_ROBOT_H
#define __UI_ROBOT_H

#include "ui.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"

class QDockWidget;

namespace mrrocpp {
namespace ui {
namespace common {

#define CATCH_SECTION_IN_ROBOT catch (ecp::common::robot::ECP_main_error & e) { \
	/* Obsluga bledow ECP */ \
		catch_ecp_main_error(e); \
  } /*end: catch */ \
\
catch (ecp::common::robot::ECP_error & er) { \
	/* Wylapywanie bledow generowanych przez modul transmisji danych do EDP */ \
		catch_ecp_error(er); \
} /* end: catch */ \
\
catch(const std::exception & e){\
	catch_std_exception(e); \
}\
\
catch (...) {  /* Dla zewnetrznej petli try*/ \
	/* Wylapywanie niezdefiniowanych bledow*/ \
		catch_tridot(); \
} /*end: catch */\


#define CATCH_SECTION_UI catch (ecp::common::robot::ECP_main_error & e) { \
	/* Obsluga bledow ECP */ \
		robot.catch_ecp_main_error(e); \
  } /*end: catch */ \
\
catch (ecp::common::robot::ECP_error & er) { \
	/* Wylapywanie bledow generowanych przez modul transmisji danych do EDP */ \
		robot.catch_ecp_error(er); \
} /* end: catch */ \
\
catch(const std::exception & e){\
	robot.catch_std_exception(e); \
}\
\
catch (...) {  /* Dla zewnetrznej petli try*/ \
	/* Wylapywanie niezdefiniowanych bledow*/ \
		robot.catch_tridot(); \
} /*end: catch */\


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

public:
	Interface& interface;

	feb_thread* tid;
	function_execution_buffer eb;

	ecp_edp_ui_robot_def state;

	boost::shared_ptr <lib::sr_ecp> msg; // Wskaznik na obiekt do komunikacji z SR z funkcja ECP dla wszystkich robotow

	/**
	 * @brief Unique robot name
	 */
	const lib::robot_name_t robot_name; // by Y - nazwa robota (track, postument etc.)
	int number_of_servos;
	//std::string activation_string;

	common::WndBase_t wndbase_m;

	UiRobot(Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos);

	void create_thread();
	void abort_thread();
	void pulse_reader_execute(int code, int value);

	bool pulse_reader_start_exec_pulse(void);
	bool pulse_reader_stop_exec_pulse(void);
	bool pulse_reader_trigger_exec_pulse(void);

	void pulse_ecp(void);
	void close_all_windows();
	void EDP_slay_int();
	void close_edp_connections();
	void connect_to_reader(void);
	void connect_to_ecp_pulse_chanell(void); //TODO: channel, nie chanell
	void pulse_ecp_execute(int code, int value);
	virtual void delete_ui_ecp_robot() = 0;
	virtual void null_ui_ecp_robot() = 0;
	virtual int ui_get_edp_pid() = 0;
	virtual void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l) = 0;
	virtual int manage_interface() = 0;

	virtual int synchronise() = 0;
	virtual void edp_create();
	virtual int edp_create_int();
	virtual int create_ui_ecp_robot() = 0;
	virtual int edp_create_int_extra_operations();

	bool check_synchronised_and_loaded();
	bool deactivate_ecp_trigger();
	int reload_configuration();

	virtual int move_to_synchro_position();
	virtual int move_to_front_position();
	virtual int move_to_preset_position(int variant);

	// default try catch handlers
	void catch_ecp_main_error(ecp::common::robot::ECP_main_error & e);
	void catch_ecp_error(ecp::common::robot::ECP_error & er);
	void catch_std_exception(const std::exception & e);
	void catch_tridot();

};

}
}
}
#endif

