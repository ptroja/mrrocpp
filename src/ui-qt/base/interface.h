#ifndef __INTERFACE_H
#define __INTERFACE_H

#include <iostream>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <QMainWindow>
#include <QTimer>
#include <QVBoxLayout>
#include <QDockWidget>

//#include "mainwindow.h"
#include "wgt_process_control.h"
#include "wgt_robot_process_control.h"
#include "ui_ecp_dialogs/wgt_yes_no.h"
#include "ui_ecp_dialogs/wgt_message.h"
#include "ui_ecp_dialogs/wgt_input_integer.h"
#include "ui_ecp_dialogs/wgt_input_double.h"
#include "ui_ecp_dialogs/wgt_choose_option.h"
#include "ui_ecp_dialogs/wgt_teaching.h"

#include "base/lib/sr/sr_ecp.h"
#include "base/lib/sr/sr_ui.h"
#include "base/lib/configurator.h"
#include "base/ecp/ecp_robot.h"

//#include "string"

#include "ui.h"

#include "base/lib/messip/messip_dataport.h"

//namespace Ui{
class MainWindow;
//}

namespace mrrocpp {
namespace ui {
namespace common {

class AllRobots;
class Mp;

#define ADD_UI_ROBOT(__robot_name) \
		{\
			common::UiRobot *created_robot = new __robot_name::UiRobot(*this);\
			robot_m[created_robot->robot_name] = created_robot;\
		}

class UiRobot;

typedef std::map <lib::robot_name_t, UiRobot*> robots_t;
typedef robots_t::value_type robot_pair_t;

class sr_buffer;
class ecp_buffer;

// super klasa agregujaca porozrzucane struktury

class Interface : public QObject
{
Q_OBJECT
private:

	void create_robots();
	boost::shared_ptr <QTimer> timer;

	bool html_it(std::string &_input, std::string &_output);

	void setRobotsMenu();

signals:
	void manage_interface_signal();
	void raise_process_control_window_signal();
	void raise_ui_ecp_window_signal();

private slots:

	void manage_interface_slot();
	void raise_process_control_window_slot();
	void raise_ui_ecp_window_slot();
	void timer_slot();

public:

	Interface();

	~Interface();

	void raise_process_control_window();
	void raise_ui_ecp_window();
	void start_on_timer();

	boost::shared_ptr <MainWindow> mw;

	//static Interface * get_instance();
	MainWindow* get_main_window() const;
	wgt_process_control* get_process_control_window();

	void print_on_sr(const char *buff, ...);

	busy_flag communication_flag;

	//! pointers to threads
	boost::shared_ptr <sr_buffer> ui_sr_obj;
	boost::shared_ptr <ecp_buffer> ui_ecp_obj;
	boost::shared_ptr <feb_thread> meb_tid;

	function_execution_buffer *main_eb;

	typedef std::string list_t;

	// listy sekcji i wezlow sieciowych plikow konfiguracyjnych
	std::list <list_t> section_list, config_node_list, all_node_list;
	// lista nazw programow i wezlow na ktorych maja byc uruchamiane
	std::list <program_node_user_def> program_node_user_list;

	// TODO: change to ENUM
	short ui_state;// 1 working, 2 exiting started, 3-5 exiting in progress - mrrocpp processes closing, 6 - exit imeditily

	TEACHING_STATE teachingstate;// dawne systemState do nauki
	TEACHING_STATE_ENUM file_window_mode;
	UI_NOTIFICATION_STATE_ENUM notification_state, next_notification;

	std::ofstream *log_file_outfile;

	boost::mutex process_creation_mtx;
	boost::mutex ui_notification_state_mutex;
	boost::shared_ptr <lib::configurator> config;
	boost::shared_ptr <lib::sr_ui> ui_msg;// Wskaznik na obiekt do komunikacji z SR

	// bool is_any_edp_active;
	bool is_mp_and_ecps_active;

	const int position_refresh_interval;

	int set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion);
	void UI_close(void);
	void init();

	bool check_sigchld_handling();
	void block_sigchld();
	void unblock_sigchld();
	void mask_signals_for_thread();
	int wait_for_child_termination(pid_t pid, bool hang);
	int sigchld_handling;
	int manage_interface(void);
	void manage_pc(void);

	void reload_whole_configuration();

	//! @bug: this call is not used. It should be deleted, since
	//! thread objects are managed with boost::shared_ptr and deleted
	//! automatically when a container object is deleted.
	void abort_threads();
	void fill_node_list(void);
	int fill_section_list(const char *file_name_and_path);
	int initiate_configuration(void);
	int clear_all_configuration_lists(void);
	int fill_program_node_list(void);
	int get_default_configuration_file_name(void);
	int set_default_configuration_file_name(void);
	int check_edps_state_and_modify_mp_state(void);
	int check_gns(void);
	bool check_node_existence(const std::string & _node, const std::string & beginnig_of_message);

	//! TODO: throw an exception (assumed inheritance from std::exception)

	std::string config_file_relativepath;// sciezka lokalana do konfiguracji wraz z plikiem konfiguracyjnym
	std::string binaries_network_path;// sieciowa sciezka binariow mrrocpp
	std::string binaries_local_path;// lokalna sciezka binariow mrrocpp
	std::string mrrocpp_local_path;// lokalna sciezka mrrocpp: np. "/home/yoyek/mrrocpp/build". W niej katalogi bin, configs etc.
	std::string mrrocpp_root_local_path;// lokalna sciezka (bez build) mrrocpp: np. "/home/yoyek/mrrocpp". W niej katalogi bin, configs etc.

	std::string teach_filesel_fullpath;// sciezka domyslana dla fileselect dla generatora uczacego
	std::string config_file;// nazwa pliku konfiguracyjnego dla UI
	std::string session_name;// nazwa sesji
	std::string config_file_fullpath;// sciezka globalna do konfiguracji

	std::string ui_attach_point;
	std::string network_sr_attach_point;
	std::string sr_attach_point;
	std::string ui_node_name;// nazwa wezla na ktorym jest uruchamiany UI

	const std::string mrrocpp_bin_to_root_path;

	// The Ui robots

	/**
	 * @brief map of all robots used in the task
	 */

	common::robots_t getRobots() const;

	common::robots_t robot_m;

	void create_threads();

	int unload_all();
	int slay_all();

	Mp *mp;
	AllRobots *all_robots;

	void open_process_control_windows();

	wgt_process_control* get_wgt_pc()
	{
		return wgt_pc;
	}

	// windows

	wgt_process_control* wgt_pc;
	wgt_yes_no* wgt_yes_no_obj;
	wgt_message* wgt_message_obj;
	wgt_input_integer* wgt_input_integer_obj;
	wgt_input_double* wgt_input_double_obj;
	wgt_choose_option* wgt_choose_option_obj;
	wgt_teaching* wgt_teaching_obj;

};

}
}
}

#endif

