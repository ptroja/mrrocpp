// -------------------------------------------------------------------------
// Proces:		EFFECTOR CONTROL PROCESS (ECP)
// Plik:			ecp_ftg.h
// System:	QNX/MRROC++  v. 6.3
// Opis:		force_controlled_trajectory_generator - deklaracja klasy
// 				generator do uczenia trajektorii, ruch robotem za pomoca czujnika sily
// Autor:		tkornuta
// Data:		04.11.2005
// -------------------------------------------------------------------------

#if !defined(_ECP_TRG_H)
#define _ECP_TRG_H

#include <list>

#include "common/impconst.h"
#include "common/com_buf.h"

// Naglowek zawierajacy klase ecp_mp_force_sensor.
#include "ecp_mp/ecp_mp_s_force.h"
#include "ecp/common/ecp_teach_in_generator.h"

// ####################################################################
// ###########  KLASA do ruszania robotem, ruch kontrolowany za  pomoca czujnika sily    #########
// ####################################################################

class force_controlled_trajectory_generator : public ecp_teach_in_generator
{
	private:
		// Lista pozycji dla danego makrokroku - ruch jedna osia/wspolrzedna.
		std::list<double> position_list;
		std::list<double>::iterator position_list_iterator;
		// Metody zwiazane z lista pozycji jednej osi/wspolrzednej.
		void flush_position_list (void);
		void initiate_position_list (void);
		void next_position_list_element (void);
		void get_position_list_element (double &position);
		bool is_position_list_element ( void );
		void create_position_list_head (double position);
		void insert_position_list_element (double position);
		// ~Aktualna pozycja - w roznych wspolrzednych.
		double current_position[8];
		// ~Aktualna pozycja - na motorach.
		double current_motor_position[8];
		// Standardowe przesuniecia na motorach.
		double motor_delta[8];
		// Przyrost przesuniecia na motorach.
		double motor_delta_increment[8];
		// Maksymalny przyrost przesuniecia na motorach.
		double motor_max_delta_increment[8];
		// Standardowe przesuniecia na wspolrzednych zewnetrznych.
		double external_delta[6];
		// Przyrost przesuniecia na wspolrzednych zewnetrznych.
		double external_delta_increment[6];
		// Maksymalny przyrost przesuniecia na wspolrzednych zewnetrznych.
		double external_max_delta_increment[6];
		// Standardowe przesuniecia - aktualnie uzywane.
		double current_delta[8];
		// Przyrost przesuniecia - aktualnie uzywane.
		double current_delta_increment[8];
		// Maksymalny przyrost - aktualnie uzywane.
		double current_max_delta_increment[8];
		// Obecne sterowanie.
		POSE_SPECIFICATION current_control;
		// Numer osi/wspolrzednej, kierunek - dane z przetworzonego polecenia ruchu.
		short number, dir;
		// Sila przy ktorej nalezy przerwac ruch.
		short dangerous_force;
		// Zwiekszanie wielkosci kroku.
		bool increment_delta(double &tmp_delta, double direction, double max_delta_increment, double delta_increment);
		// Sprawdzenie warunku rozpoczecia zwalniania.
		bool slow_down_condition(double start_position, double after_acceleration_position, double current_position, double stop_position);
		// Zmniejszanie wielkosci kroku.
		bool decrement_delta(double &tmp_delta, double direction, double delta_increment);
		// Sprawdzenie odczytow z czujnika sily.
		void check_force_condition(ecp_mp_force_sensor& the_sensor);

		// Mutex do odbierania pozycji robota
		pthread_mutex_t ROBOT_POSITION_MUTEX;
	public:
		force_controlled_trajectory_generator (ecp_task& _ecp_task);
		~force_controlled_trajectory_generator (void);
		// Dodanie nowego kroku do listy.
		void add_step(int motion_time);
		// Zapisanie trajektorii.
		void save_trajectory(char* filename);
		// Ustawienie rodzaju ruchu.
		void set_move_type(short move_type);
		// Przygotowanie trajektorii do wykonania.
		virtual bool first_step ();
		// Wykonanie wlasciwej trajektorii.
		virtual bool next_step ();
		// Zwraca aktualnie posiadana pozycje.
		void return_position (double robot_position[8]);
		// Zwraca aktualnie posiadana pozycje na motorach.
		void return_motor_position (double robot_position[8]);
		// Zwraca aktualne odczyty z czujnika sily.
		void return_sensor_reading(ecp_mp_force_sensor& the_sensor, double sensor_reading[6]);
		// Odczyt pozycji robota z EDP.
		void get_current_position ();
		// Zmiana sterowania MOTOR/EXTERNAL.
		void change_control(POSE_SPECIFICATION ps);
		// Ustawienie narzedzia - czujnika przymocowanego do kolnierza.
		void set_tool_frame();
		// Ustawienie pustej macierzy narzedzia.
		void reset_tool_frame();
		// Odczyt wielkosci niebezpiecznej sily z pliku INI.
		void set_dangerous_force();
}
; // end: class force_controlled_trajectory_generator

#endif
