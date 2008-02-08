#ifndef MP_TASK_H_
#define MP_TASK_H_

#include "mp/mp_generator.h"
#include "mp/mp_robot.h"
#include "mp/mp.h"

#include <list>
#include <map>

// klasa globalna dla calego procesu MP
class mp_task: public ecp_mp_task
{
	protected:
		/// lista generatorow dla scheduler'a
		std::list<mp_generator*> gen_list;
		std::list<mp_generator*>::iterator gen_list_iterator;

		bool all_gen_sets_waiting_for_ECP_pulse;

	public:
		static name_attach_t *mp_trigger_attach;
		static name_attach_t *mp_attach;

		/// mapa wszystkich robotow
		static std::map <ROBOT_ENUM, mp_robot*> robot_m;

		int32_t ui_scoid; // server connection id
		char ui_pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
		bool ui_new_pulse; // okresla czy jest nowy puls

		/// KONSTRUKTORY
		mp_task(void);
		virtual ~mp_task(void);

		// METODY
		// funkcja odbierajaca pulsy z ECP

		// -------------------------------------------------------------------
		// inicjacja polaczen, rejestracja nazwy MP, odszukanie UI, SR by Y&W
		// -------------------------------------------------------------------
		void mp_initialize_communication (void);

		// oczekiwanie na puls z ECP
		enum MP_RECEIVE_PULS_ENUM {
			WITH_TIMEOUT,
			WITHOUT_TIMEOUT
		};

		enum WAIT_FOR_NEW_PULSE_ENUM {
		    NEW_ECP_PULSE,
		    NEW_UI_PULSE,
		    NEW_UI_OR_ECP_PULSE
		};

		bool set_next_ecps_state (int l_state, int l_variant, char* l_string, int number_of_robots, ... );
		bool send_end_motion_to_ecps (int number_of_robots, ... );
		bool run_ext_empty_gen (bool activate_trigger, int number_of_robots, ... );
		bool run_ext_empty_gen_for_set_of_robots_and_wait_for_task_termin_mess_of_another_set_of_robots
		(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ... );
		bool wait_ms (int _ms_delay); // zamiast delay

		int mp_receive_pulse (mp_receive_pulse_struct_t* outputs, MP_RECEIVE_PULS_ENUM tryb);
		int check_and_optional_wait_for_new_pulse (mp_receive_pulse_struct_t* outputs,
		        WAIT_FOR_NEW_PULSE_ENUM process_mode, MP_RECEIVE_PULS_ENUM desired_wait_mode);
		int mp_wait_for_ui_name_open(void);

		// mp_receive_ecp_pulse_return_t mp_receive_ecp_pulse (int tryb);
		// oczekwianie na name_open do kanalu do przesylania pulsow miedzy ECP i MP
		// int mp_wait_for_name_open_ecp_pulse (mp_receive_pulse_struct_t* outputs, uint32_t nd, pid_t ECP_pid);
		int mp_wait_for_name_open_ecp_pulse(mp_receive_pulse_struct_t* outputs);

		// Oczekiwanie na zlecenie START od UI
		void wait_for_start (void);// by Y&W

		// Oczekiwanie na zlecenie STOP od UI
		void wait_for_stop ( WAIT_FOR_STOP_ENUM tryb);// by Y&W dodany tryb

		// Wystartowanie wszystkich ECP
		void start_all (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);

		// Zatrzymanie wszystkich ECP
		void terminate_all (std::map <ROBOT_ENUM, mp_robot*>& _robot_m );

		// Wyslanie rozkazu do wszystkich ECP
		void execute_all (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);

		// funkcja odbierajaca pulsy z UI wykorzystywana w MOVE
		bool mp_receive_ui_pulse (std::map <ROBOT_ENUM, mp_robot*>& _robot_m, short* trigger);

		// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE
		bool mp_receive_ui_or_ecp_pulse (std::map <ROBOT_ENUM, mp_robot*>& _robot_m, mp_generator& the_generator );

		// Funkcja ruchu
		bool Move ( mp_generator& the_generator );

		// dla scheduller'a
		bool add_gen (mp_generator* gen_l);
		// not used anymore (ptrojane)
		//bool rm_gen (mp_generator* gen_l);
		bool clear_gen_list (void);

		bool scheduller_run (void);

		// obsluga sygnalu
		virtual void catch_signal_in_mp_task(int sig);

		/// Zatrzymanie wszystkich ECP
		static void kill_all_ECP (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);

		/// utworzenie robotow
		virtual bool create_robots(void);

		/// methods for MP template to redefine in concrete classes
		virtual void task_initialization(void);
		virtual void main_task_algorithm(void);
};

mp_task* return_created_mp_task (void);

#endif /*MP_TASK_H_*/
