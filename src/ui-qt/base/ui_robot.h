// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_ROBOT_H
#define __UI_ROBOT_H

#include <QObject>
#include <QMenu>
#include "ui.h"
#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_robot.h"
#include "wgt_base.h"

class QDockWidget;
class wgt_robot_process_control;

namespace Ui {
class MenuBar;
class SignalDispatcher;
}

namespace mrrocpp {
namespace ui {
namespace common {

class EcpRobot;

const std::string WGT_ROBOT_PC = "WGT_ROBOT_PC";

#define CATCH_SECTION_IN_ROBOT catch (ecp::exception::se_r & error) { \
	/* Obsluga bledow ECP */ \
		catch_ecp_robot_se(error); \
  } /*end: catch */ \
\
catch (ecp::exception::nfe_r & error) { \
	/* Obsluga bledow ECP */ \
		catch_ecp_robot_nfe(error); \
 } /*end: catch */ \
\
catch (ecp::exception::fe_r & error) { \
	/* Obsluga bledow ECP */ \
		catch_ecp_robot_fe(error); \
 } /*end: catch */ \
\
catch(const std::exception & e){\
	catch_std_exception(e); \
}\
\
catch (...) {  /* Dla zewnetrznej petli try*/ \
	/* Wylapywanie niezdefiniowanych bledow*/ \
		catch_tridot(); \
} /*end: catch */\


#define CATCH_SECTION_UI_PTR catch (ecp::exception::se_r & error) { \
	/* Obsluga bledow ECP */ \
		robot->catch_ecp_robot_se(error); \
  } /*end: catch */ \
\
catch (ecp::exception::fe_r & error) { \
	/* Obsluga bledow ECP */ \
		robot->catch_ecp_robot_fe(error); \
 } /*end: catch */ \
\
catch (ecp::exception::nfe_r & error) { \
	/* Obsluga bledow ECP */ \
		robot->catch_ecp_robot_nfe(error); \
 } /*end: catch */ \
\
catch(const std::exception & e){\
	robot->catch_std_exception(e); \
}\
\
catch (...) {  /* Dla zewnetrznej petli try*/ \
	/* Wylapywanie niezdefiniowanych bledow*/ \
		robot->catch_tridot(); \
} /*end: catch */\


class Interface;

//
//
// KLASA UiRobot
//
//

// super klasa agregujaca porozrzucane dotychczas struktury

class UiRobot : public QObject
{
	Q_OBJECT

public:

	EcpRobot *ui_ecp_robot;
	Interface& interface;

	const lib::robot_name_t getName();

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

	//common::WndBase_t wndbase_m;

	UiRobot(Interface& _interface, lib::robot_name_t _robot_name, int _number_of_servos);
	~UiRobot();

	/*
	 * opens move window on mp or ecp request
	 * C_XYZ_ANGLE_AXIS variant
	 */

	virtual void open_c_xyz_angle_axis_window();

	/*
	 * opens move window on mp or ecp request
	 * 	 * C_XYZ_EULER_ZYZ variant
	 */

	virtual void open_c_xyz_euler_zyz_window();

	/*
	 * opens move window on mp or ecp request
	 * 	 * C_JOINT variant
	 */

	virtual void open_c_joint_window();

	/*
	 * opens move window on mp or ecp request
	 * 	 * C_MOTOR variant
	 */

	virtual void open_c_motor_window();

	bool is_edp_loaded();

	void create_thread();
	void abort_thread();
	void pulse_reader_execute(int code, int value);

	bool pulse_reader_start_exec_pulse(void);
	bool pulse_reader_stop_exec_pulse(void);
	bool pulse_reader_trigger_exec_pulse(void);

	bool is_process_control_window_created();
	void indicate_process_control_window_creation();

	void pulse_ecp(void);
	void close_all_windows();
	void EDP_slay_int();
	void abort_edp();
	void connect_to_reader(void);
	void connect_to_ecp_pulse_channel(void);
	void pulse_ecp_execute(int code, int value);
	virtual void delete_ui_ecp_robot();
	pid_t ui_get_edp_pid() const;
	void ui_get_controler_state(lib::controller_state_t & robot_controller_initial_state_l);
	virtual void manage_interface();
	virtual void setup_menubar();
	virtual int execute_clear_fault()
	{
		return 0;
	}

	virtual double* getCurrentPos()
	{
		return NULL;
	}
	virtual double* getDesiredPos()
	{
		return NULL;
	}

	virtual void synchronise() = 0;
	virtual void edp_create();
	virtual int edp_create_int();
	virtual void create_ui_ecp_robot() = 0;

	wgt_robot_process_control * get_wgt_robot_pc();
	void delete_robot_process_control_window();
	void open_robot_process_control_window();

	void block_ecp_trigger();
	void unblock_ecp_trigger();

	virtual void edp_create_int_extra_operations();

	bool check_synchronised_and_loaded();
	bool deactivate_ecp_trigger();
	void reload_configuration();

	virtual void move_to_synchro_position();
	virtual void move_to_front_position();
	virtual void move_to_preset_position(int variant);

	// default try catch handlers
	void catch_ecp_robot_fe(ecp::exception::fe_r & error);
	void catch_ecp_robot_se(ecp::exception::se_r & error);
	void catch_ecp_robot_nfe(ecp::exception::nfe_r & error);
	void catch_std_exception(const std::exception & e);
	void catch_tridot();

	//void

	typedef void (UiRobot::*uiRobotFunctionPointer)();
	typedef void (UiRobot::*uiRobotFunctionPointerInt)(int);
	typedef int (UiRobot::*intUiRobotFunctionPointerInt)(int);
	typedef int (UiRobot::*intUiRobotFunctionPointer)();

	typedef std::map <std::string, wgt_base*> wgt_t;
	typedef wgt_t::value_type wgt_pair_t;

	wgt_robot_process_control *wgt_robot_pc;

	typedef std::map <lib::robot_name_t, UiRobot*> robots_t;
	wgt_t wgts;

	bool process_control_window_created;

	void zero_desired_position();

	template <typename T> void add_wgt(std::string name, QString label)
	{
		wgt_base *created_wgt = new T(label, interface, this);
		wgts[name] = created_wgt;
	}

	double *current_pos; // pozycja biezaca
	double *desired_pos; // pozycja zadana

protected:
	QAction *EDP_Load;
	QAction *EDP_Unload;
	QAction *wgt_robot_process_control_action;

	QMenu *robot_menu;
};

}
}
}
#endif

