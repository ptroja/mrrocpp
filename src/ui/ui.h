// -------------------------------------------------------------------------
//                            ui.h
// Definicje struktur danych i metod dla procesu UI
//
// Ostatnia modyfikacja: 16.04.98
// -------------------------------------------------------------------------

#ifndef __UI_H
#define __UI_H

#include <pthread.h>
#include <list>

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#include "lib/com_buf.h"
#include "lib/srlib.h"
#include "lib/mis_fun.h"

#define CATCH_SECTION_UI catch (ecp::common::ecp_robot::ECP_main_error e) { \
	/* Obsluga bledow ECP */ \
	if (e.error_class == lib::SYSTEM_ERROR) \
		printf("ECP lib::SYSTEM_ERROR error in UI\n"); \
		ui_state.ui_state=2; \
	/*  exit(EXIT_FAILURE);*/ \
  } /*end: catch */ \
\
catch (ecp::common::ecp_robot::ECP_error er) { \
	/* Wylapywanie bledow generowanych przez modul transmisji danych do EDP */ \
	if ( er.error_class == lib::SYSTEM_ERROR) { /* blad systemowy juz wyslano komunikat do SR */ \
		perror("ECP lib::SYSTEM_ERROR in UI"); \
		/* PtExit( EXIT_SUCCESS ); */ \
	} else { \
	switch ( er.error_no ) { \
		case INVALID_POSE_SPECIFICATION: \
		case INVALID_ECP_COMMAND: \
		case INVALID_COMMAND_TO_EDP: \
		case EDP_ERROR: \
		case INVALID_EDP_REPLY: \
		case INVALID_RMODEL_TYPE: \
			/* Komunikat o bledzie wysylamy do SR */ \
			ui_msg.all_ecp->message (lib::NON_FATAL_ERROR, er.error_no); \
		break; \
		default: \
			ui_msg.all_ecp->message (lib::NON_FATAL_ERROR, 0, "ECP: Unidentified exception"); \
			perror("Unidentified exception"); \
		} /* end: switch */ \
	} \
} /* end: catch */ \
\
catch (...) {  /* Dla zewnetrznej petli try*/ \
	/* Wylapywanie niezdefiniowanych bledow*/ \
	/* Komunikat o bledzie wysylamy do SR (?) */ \
	fprintf(stderr, "unidentified error in UI\n"); \
} /*end: catch */\


enum TEACHING_STATE_ENUM {
	FSTRAJECTORY, FSCONFIG
};

enum UI_NOTIFICATION_STATE_ENUM {
	UI_N_STARTING,
	UI_N_READY,
	UI_N_BUSY,
	UI_N_EXITING,
	UI_N_COMMUNICATION,
	UI_N_PROCESS_CREATION,
	UI_N_SYNCHRONISATION
};

// FIXME: moved from proto.h for linux compatibility
int set_ui_state_notification(UI_NOTIFICATION_STATE_ENUM new_notifacion);

enum UI_ECP_COMMUNICATION_STATE {
	UI_ECP_AFTER_RECEIVE, UI_ECP_REPLY_READY, UI_ECP_AFTER_REPLY
};

enum UI_MP_STATE {
	UI_MP_NOT_PERMITED_TO_RUN,
	UI_MP_PERMITED_TO_RUN,
	UI_MP_WAITING_FOR_START_PULSE,
	UI_MP_TASK_RUNNING,
	UI_MP_TASK_PAUSED
};

enum UI_ALL_EDPS_STATE {
	UI_ALL_EDPS_NONE_EDP_ACTIVATED,
	UI_ALL_EDPS_NONE_EDP_LOADED,
	UI_ALL_EDPS_THERE_IS_EDP_LOADED_BUT_NOT_ALL_ARE_LOADED,
	UI_ALL_EDPS_LOADED_BUT_NOT_SYNCHRONISED,
	UI_ALL_EDPS_LOADED_AND_SYNCHRONISED
};

// -1 mp jest wylaczone i nie moze zostac wlaczone , 0 - mp wylaczone ale wszystkie edp gotowe,  1- wlaczone czeka na start
// 2 - wlaczone czeka na stop 3 -wlaczone czeka na resume


// czas jaki uplywa przed wyslaniem sygnalu w funkcji ualarm w mikrosekundach
#define SIGALRM_TIMEOUT 1000000

typedef struct {
	pid_t pid;
	int test_mode;
	std::string node_name;
	std::string section_name; // nazwa sekcji, w ktorej zapisana jest konfiguracja
	std::string network_resourceman_attach_point;
	std::string hardware_busy_attach_point; // do sprawdzenie czy edp juz nie istnieje o ile nie jest tryb testowy
	std::string network_reader_attach_point;
	int node_nr;
	int reader_fd;
	bool is_synchronised;
	int state; // -1, edp nie aktywne, 0 - edp wylaczone 1- wlaczone czeka na reader start 2 - wlaczone czeka na reader stop
	int last_state;
	std::string preset_sound_0; // dla EDP speaker
	std::string preset_sound_1;
	std::string preset_sound_2;
	double preset_position[3][MAX_SERVOS_NR]; // pozycje zapisane w konfiguracji
	double front_position[MAX_SERVOS_NR];
} edp_state_def;

typedef struct {
	pid_t pid;
	std::string node_name;
	std::string section_name; // nazwa sekcji, w ktorej zapisana jest konfiguracja
	std::string network_trigger_attach_point;
	int node_nr;
	int trigger_fd;
	int state;
	int last_state;
} ecp_state_def;

typedef struct {
	bool is_active;
	edp_state_def edp;
	ecp_state_def ecp;
} ecp_edp_ui_robot_def;

typedef struct {
	pid_t pid;
	std::string node_name;
	std::string network_pulse_attach_point;
	int node_nr;
	int pulse_fd;
	UI_MP_STATE state;
	UI_MP_STATE last_state;
} mp_state_def;

typedef struct {
	std::string program_name;
	std::string node_name;
} program_node_def;

typedef struct {

	UI_ALL_EDPS_STATE all_edps;
	std::string binaries_network_path; // sieciowa sciezka binariow mrrocpp
	std::string binaries_local_path; // lokalna sciezka binariow mrrocpp
	std::string mrrocpp_local_path; // lokalna sciezka mrrocpp: np. "/home/yoyek/mrrocpp/". W niej katalogi bin, configs etc.

	std::string teach_filesel_fullpath; // sciezka domyslana dla fileselect dla generatora uczacego
	std::string config_file;// nazwa pliku konfiguracyjnego dla UI
	std::string session_name; // nazwa sesji
	std::string config_file_fullpath; // sciezka globalna do konfiguracji
	std::string config_file_relativepath; // sciezka lokalana do konfiguracji wraz z plikiem konfiguracyjnym

	std::string ui_attach_point;
	std::string network_sr_attach_point;
	std::string sr_attach_point;

	typedef std::string list_t;

	// listy sekcji i wezlow sieciowych plikow konfiguracyjnych
	std::list<list_t> section_list, config_node_list, all_node_list;
	// lista nazw programow i wezlow na ktorych maja byc uruchamiane
	std::list<program_node_def> program_node_list;

	std::string ui_node_name; // nazwa wezla na ktorym jest uruchamiany UI
	int ui_node_nr; // numer wezla na ktorym jest uruchamiany UI
	pid_t ui_pid; // pid UI
	short ui_state; // 1 working, 2 exiting started, 3-5 exiting in progress - mrrocpp processes closing, 6 - exit imeditily

	ecp_edp_ui_robot_def irp6_on_track;
	ecp_edp_ui_robot_def irp6_postument;
	ecp_edp_ui_robot_def irp6_mechatronika;
	ecp_edp_ui_robot_def polycrank;
	ecp_edp_ui_robot_def conveyor;
	ecp_edp_ui_robot_def speaker;

	mp_state_def mp;
	// bool is_any_edp_active;
	bool is_mp_and_ecps_active;
	bool is_sr_thread_loaded; // informacja czy okno zadania jest otwarte

	int teachingstate; // dawne systemState do nauki
	TEACHING_STATE_ENUM file_window_mode;
	UI_NOTIFICATION_STATE_ENUM notification_state;

	bool is_task_window_open; // informacja czy okno zadania jest otwarte
	bool is_process_control_window_open; // informacja czy okno sterowania procesami jest otwarte
	bool process_control_window_renew; // czy okno ma zostac odswierzone

	bool is_wind_irp6ot_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_irp6p_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_irp6m_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte
	bool is_wind_polycrank_int_open; // informacja czy okno ruchow w radianach stawow jest otwarte

	bool is_wind_irp6ot_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte
	bool is_wind_irp6p_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte
	bool is_wind_irp6m_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte
	bool is_wind_polycrank_inc_open; // informacja czy okno ruchow w radianach na wale silnika jest otwarte

	bool is_wind_irp6ot_xyz_euler_zyz_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_euler_zyz_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6m_xyz_euler_zyz_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_xyz_angle_axis_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_angle_axis_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6m_xyz_angle_axis_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_xyz_aa_relative_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_aa_relative_open; // informacja czy okno ruchow we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_xyz_angle_axis_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_angle_axis_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6m_xyz_angle_axis_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_xyz_euler_zyz_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6p_xyz_euler_zyz_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte
	bool is_wind_irp6m_xyz_euler_zyz_ts_open; // informacja czy okno definicji narzedzia we wspolrzednych zewnetrznych jest otwarte

	bool is_wind_irp6ot_kinematic_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6p_kinematic_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6m_kinematic_open; // informacja czy okno definicji kinematyki jest otwarte

	bool is_wind_irp6ot_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6p_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_irp6m_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte
	bool is_wind_conv_servo_algorithm_open; // informacja czy okno definicji kinematyki jest otwarte

	bool is_wind_conveyor_moves_open; // informacja czy okno ruchow dla robota conveyor

	bool is_wind_speaker_play_open; // informacja czy okno odtwarzania dzwiekow jest otwarte

	bool is_teaching_window_open; // informacja czy okno nauki jest otwarte
	bool is_file_selection_window_open; // informacja czy okno z wyborem pliku jest otwarte
} ui_state_def;

/**************************** ui_sr_buffer *****************************/

#define UI_SR_BUFFER_LENGHT 50

class ui_sr_buffer {
private:
	boost::circular_buffer<lib::sr_package_t> cb;
	boost::mutex mtx; // = PTHREAD_MUTEX_INITIALIZER ;

public:

	ui_sr_buffer();

	void put_one_msg(const lib::sr_package_t& new_msg); // podniesienie semafora
	void get_one_msg(lib::sr_package_t& new_msg); // podniesienie semafora
	bool buffer_empty(); // czy bufor cykliczny jest pusty
};

/**************************** ui_sr_buffer *****************************/

#define UI_SR_BUFFER_LENGHT 50

class ui_ecp_buffer {
private:


public:
	UI_ECP_COMMUNICATION_STATE communication_state;
	lib::ECP_message ecp_to_ui_msg;
	lib::UI_reply ui_rep;

	lib::boost_condition_synchroniser synchroniser;
	ui_ecp_buffer();

};

typedef struct {
	lib::sr_ecp* all_ecp; // Wskaznik na obiekt do komunikacji z SR z fukcja ECP dla wszystkich robotow
	lib::sr_ui* ui; // Wskaznik na obiekt do komunikacji z SR
} ui_msg_def;

void UI_close(void);

class function_execution_buffer {
public:
	typedef boost::function<int()> command_function_t;

	int wait_and_execute();
	void command(command_function_t _com_fun);
	function_execution_buffer();
private:
	boost::condition_variable cond; //! active command condition
	boost::mutex mtx; //! mutex related to condition variable

	bool has_command; //! flag indicating active command to execute

	command_function_t com_fun; //! command functor
};

// forward declaration
class busy_flag;

class busy_flagger {
private:
	//! flag object to decrement in destructor
	busy_flag & flag;

public:
	//! increment busy flag for in a  scoped manner
	busy_flagger(busy_flag & _flag);

	//! desctructor makes flag unbusy
	~busy_flagger();
};

class busy_flag {
	friend class busy_flagger;

private:
	//! count busy flagging
	int counter;

	//! guard counter variable
	mutable boost::mutex m_mutex;

	//! increment counter
	void increment();

	//! decrement counter
	void decrement();

public:
	bool is_busy() const;

	//! constructor
	busy_flag();
};

// TODO: reimplement this as a singleton
extern busy_flag communication_flag;

void create_threads();
void abort_threads();

#endif

