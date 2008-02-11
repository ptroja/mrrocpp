#ifndef MP_ROBOT_H_
#define MP_ROBOT_H_

#include "mp/mp_task.h"

class mp_robot
{
		// Klasa bazowa dla robotow (klasa abstrakcyjna)
		// Kazdy robot konkretny (wyprowadzony z klasy bazowej)
		// musi zawierac pola danych (skladowe) dotyczace
		// ostatnio zrealizowanej pozycji oraz pozycji zadanej
	protected:
		MP_COMMAND_PACKAGE mp_command;      // Bufor z rozkazem dla ECP
		// - uzytkownik nie powinien z tego korzystac
		ECP_REPLY_PACKAGE ecp_reply;        // Bufor z odpowiedzia z ECP
		// - uzytkownik nie powinien z tego korzystac

		mp_task* mp_object;

	public:
		sr_ecp* sr_ecp_msg;    // by Y - Wskaznik na obiekt do komunikacji z SR

		ROBOT_ENUM robot_name;
		
		//! deskryptor wezla na ktorym jest powolane ECP oraz jego PID (dla mp_task::kill_all_ECP)
		uint32_t nd;
		pid_t ECP_pid;
		
		int32_t scoid; // server connection id
		int ECP_fd;	// deskryptor do komunikacji z ECP
		char pulse_code; // kod pulsu ktory zostal wyslany przez ECP w celu zgloszenia gotowosci do komunikacji (wartosci w impconst.h)
		bool new_pulse; // okresla czy jest nowy puls
		bool robot_new_pulse_checked; // okresla czy czy nowy puls zostal juz uwzgledniony w generatorze
		bool communicate; // okresla czy robot ma byc obslugiwany w mp_task::execute_all

		robot_ECP_transmission_data ecp_td; // Obraz danych robota wykorzystywanych przez generator
		// - do uzytku uzytkownika (generatora)

		mp_robot (ROBOT_ENUM l_robot_name, const char* _section_name, mp_task* mp_object_l);

		class MP_error
		{  // Klasa obslugi bledow robotow
			public:
				const uint64_t error_class;
				const uint64_t mp_error;
				MP_error (uint64_t err0, uint64_t err1);
		};

		// Zlecenie wykonania ruchu przez robota
		// (realizowane przez klase konkretna):
		// na poziomie MP jest to polecenie dla ECP.
		virtual void execute_motion (void);

		// Zlecenie zakonczenia ruchu przez robota
		// (realizowane przez klase konkretna):
		// na poziomie MP jest to polecenie dla ECP.
		virtual void terminate_ecp (void);

		virtual void start_ecp ( void );

		// wypelnia bufor wysylkowy do EDP na podstawie danych zawartych w swych skladowych
		// Ten bufor znajduje sie w robocie
		virtual void create_command ( void );

		virtual void create_next_pose_command (void)
		{}

		// pobiera z pakietu przeslanego z EDP informacje i wstawia je do odpowiednich swoich skladowych
		virtual void get_reply ( void )
		{}
}
; // end: class mp_robot

#endif /*MP_ROBOT_H_*/
