#ifndef MP_TASK_H_
#define MP_TASK_H_

#include "mp/mp_generator.h"
#include "mp/mp.h"

#include <list>
#include <map>



// klasa globalna dla calego procesu MP
class mp_task: public ecp_mp::task::ecp_mp_task
{
	public:
#if !defined(USE_MESSIP_SRR)
		static name_attach_t *mp_pulse_attach;
		static name_attach_t *mp_attach;
#else
		static messip_channel_t *mp_pulse_attach;
		static messip_channel_t *mp_attach;
#endif

		/// mapa wszystkich robotow
		static std::map <ROBOT_ENUM, mp_robot*> robot_m;

		/// KONSTRUKTORY
		mp_task(configurator &_config);
		virtual ~mp_task(void);

		void initialize_communication (void);
		void stop_and_terminate (void);

		// oczekiwanie na puls z ECP
		enum MP_RECEIVE_PULSE_MODE {
			WITH_TIMEOUT,
			WITHOUT_TIMEOUT
		};

		enum WAIT_FOR_NEW_PULSE_ENUM {
		    NEW_ECP_PULSE,
		    NEW_UI_PULSE,
		    NEW_UI_OR_ECP_PULSE
		};

		void set_next_playerpos_goal (ROBOT_ENUM robot_l, const playerpos_goal_t &goal);

		void set_next_ecps_state (int l_state, int l_variant, const char* l_string, int number_of_robots, ... );

		void send_end_motion_to_ecps (int number_of_robots, ... );
		void send_end_motion_to_ecps (int number_of_robots, ROBOT_ENUM *properRobotsSet);

		void run_extended_empty_gen (bool activate_trigger, int number_of_robots, ... );
		void run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ... );
		void run_extended_empty_generator_for_set_of_robots_and_wait_for_task_termination_message_of_another_set_of_robots
		(int number_of_robots_to_move, int number_of_robots_to_wait_for_task_termin, ROBOT_ENUM *robotsToMove, ROBOT_ENUM *robotsWaitingForTaskTermination);

		void wait_ms (int _ms_delay); // zamiast delay

		// mp_receive_ecp_pulse_return_t mp_receive_ecp_pulse (int tryb);
		// oczekwianie na name_open do kanalu do przesylania pulsow miedzy ECP i MP
		int mp_wait_for_name_open(mp_receive_pulse_struct_t* outputs);

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

		// funkcja odbierajaca pulsy z UI lub ECP wykorzystywana w MOVE
		void mp_receive_ui_or_ecp_pulse (std::map <ROBOT_ENUM, mp_robot*>& _robot_m, mp_generator& the_generator );

		// obsluga sygnalu
		virtual void catch_signal_in_mp_task(int sig);

		/// Zatrzymanie wszystkich ECP
		static void kill_all_ECP (std::map <ROBOT_ENUM, mp_robot*>& _robot_m);

		/// utworzenie robotow
		virtual void create_robots(void);

		/// methods for MP template to redefine in concrete classes
		virtual void task_initialization(void);
		virtual void main_task_algorithm(void);

	private:
		int32_t ui_scoid; // server connection id
		char ui_pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
		bool ui_new_pulse; // okresla czy jest nowy puls

		int mp_receive_pulse (mp_receive_pulse_struct_t* outputs, MP_RECEIVE_PULSE_MODE tryb);
		int check_and_optional_wait_for_new_pulse (mp_receive_pulse_struct_t* outputs,
		        WAIT_FOR_NEW_PULSE_ENUM process_mode, MP_RECEIVE_PULSE_MODE desired_wait_mode);
};

mp_task* return_created_mp_task (configurator &_config);

#endif /*MP_TASK_H_*/
