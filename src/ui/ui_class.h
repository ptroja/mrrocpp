// -------------------------------------------------------------------------
//                            ui_class.h
// Definicje klasy Ui
//
// Ostatnia modyfikacja: 2010
// -------------------------------------------------------------------------

#ifndef __UI_CLASS_H
#define __UI_CLASS_H

#include "ui/ui.h"
#include "lib/configurator.h"
#include "ui/src/bird_hand/ui_r_bird_hand.h"
#include "ui/src/irp6ot_m/ui_r_irp6ot_m.h"
#include "ui/src/irp6ot_tfg/ui_r_irp6ot_tfg.h"
#include "ui/src/irp6p_m/ui_r_irp6p_m.h"
#include "ui/src/irp6p_tfg/ui_r_irp6p_tfg.h"
#include "ui/src/irp6m_m/ui_r_irp6m_m.h"
#include "ui/src/conveyor/ui_r_conveyor.h"
#include "ui/src/speaker/ui_r_speaker.h"
#include "ui/src/spkm/ui_r_spkm.h"
#include "ui/src/shead/ui_r_shead.h"
#include "ui/src/smb/ui_r_smb.h"

//
//
// KLASA ui
//
//


// super klasa agregujaca porozrzucane struktury


class Ui {
private:

public:

	ui_sr_buffer* ui_sr_obj;

	ui_ecp_buffer* ui_ecp_obj;

	feb_thread* meb_tid;

	pthread_t ui_tid;
	pthread_t sr_tid;

	function_execution_buffer main_eb;

	typedef std::string list_t;

	// listy sekcji i wezlow sieciowych plikow konfiguracyjnych
	std::list<list_t> section_list, config_node_list, all_node_list;
	// lista nazw programow i wezlow na ktorych maja byc uruchamiane
	std::list<program_node_def> program_node_list;

	int ui_node_nr; // numer wezla na ktorym jest uruchamiany UI
	pid_t ui_pid; // pid UI
	short ui_state; // 1 working, 2 exiting started, 3-5 exiting in progress - mrrocpp processes closing, 6 - exit imeditily

	int teachingstate; // dawne systemState do nauki
	TEACHING_STATE_ENUM file_window_mode;
	UI_NOTIFICATION_STATE_ENUM notification_state;

	bool is_task_window_open; // informacja czy okno zadania jest otwarte
	bool is_process_control_window_open; // informacja czy okno sterowania procesami jest otwarte
	bool process_control_window_renew; // czy okno ma zostac odswierzone

	bool is_teaching_window_open; // informacja czy okno nauki jest otwarte
	bool is_file_selection_window_open; // informacja czy okno z wyborem pliku jest otwarte

	std::ofstream *log_file_outfile;

	boost::mutex process_creation_mtx;
	lib::configurator* config;
	lib::sr_ecp* all_ecp_msg; // Wskaznik na obiekt do komunikacji z SR z fukcja ECP dla wszystkich robotow
	lib::sr_ui* ui_msg; // Wskaznik na obiekt do komunikacji z SR

	mp_state_def mp;
	// bool is_any_edp_active;
	bool is_mp_and_ecps_active;
	bool is_sr_thread_loaded;
	UI_ALL_EDPS_STATE all_edps;
	std::string config_file_relativepath; // sciezka lokalana do konfiguracji wraz z plikiem konfiguracyjnym
	std::string binaries_network_path; // sieciowa sciezka binariow mrrocpp
	std::string binaries_local_path; // lokalna sciezka binariow mrrocpp
	std::string mrrocpp_local_path; // lokalna sciezka mrrocpp: np. "/home/yoyek/mrrocpp/". W niej katalogi bin, configs etc.

	std::string teach_filesel_fullpath; // sciezka domyslana dla fileselect dla generatora uczacego
	std::string config_file;// nazwa pliku konfiguracyjnego dla UI
	std::string session_name; // nazwa sesji
	std::string config_file_fullpath; // sciezka globalna do konfiguracji


	std::string ui_attach_point;
	std::string network_sr_attach_point;
	std::string sr_attach_point;
	std::string ui_node_name; // nazwa wezla na ktorym jest uruchamiany UI


	// The Ui robots
	UiRobotBirdHand bird_hand;
	UiRobotIrp6ot_m irp6ot_m;
	UiRobotIrp6ot_tfg irp6ot_tfg;
	UiRobotIrp6p_m irp6p_m;
	UiRobotIrp6p_tfg irp6p_tfg;
	UiRobotIrp6m_m irp6m_m;
	UiRobotConveyor conveyor;
	UiRobotSpeaker speaker;
	UiRobotSpkm spkm;
	UiRobotSmb smb;
	UiRobotShead shead;

	Ui();
	void UI_close(void);
	void init();
	int manage_interface(void);
	int reload_whole_configuration();
	void abort_threads();
	int fill_node_list(void);
	int fill_section_list(const char *file_name_and_path);
	int initiate_configuration(void);
	int clear_all_configuration_lists(void);
	int fill_program_node_list(void);
	int get_default_configuration_file_name(void);
	int set_default_configuration_file_name(void);
	bool check_synchronised_or_inactive(ecp_edp_ui_robot_def &robot);
	bool check_synchronised_and_loaded(ecp_edp_ui_robot_def &robot);
	bool check_loaded_or_inactive(ecp_edp_ui_robot_def &robot);
	bool check_loaded(ecp_edp_ui_robot_def &robot);
	int check_edps_state_and_modify_mp_state(void);
	int check_gns(void);
	bool check_node_existence(const std::string _node,
			const std::string beginnig_of_message);
	bool deactivate_ecp_trigger(ecp_edp_ui_robot_def &robot_l);
	int execute_mp_pulse(char pulse_code);
	int pulse_reader_execute(int coid, int pulse_code, int pulse_value);

};

#endif

